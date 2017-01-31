#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <regex.h>
#include <gtk/gtk.h>

#define GL_GLEXT_PROTOTYPES 1
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

#include <bot_core/bot_core.h>
#include <bot_vis/gl_util.h>
#include <bot_vis/gtk_util.h>
#include <bot_vis/viewer.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_lcmgl_render/lcmgl_decode.h>

#include <geom_utils/geometry.h>
#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>

#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_lcmgl_data_t.h>

#define LCMGL_VISION_CHANNEL_REGEX "LCMGL_VISION\\..*"
// The \w doesn't work on OSX
//#define LCMGL_VISION_CHANNEL_MATCH_REGEX "LCMGL_VISION\\.(\\w+)\\.(\\w+)"
#define LCMGL_VISION_CHANNEL_MATCH_REGEX "LCMGL_VISION\\.([^\\.]+)\\.([^\\.]+)"


#define RENDERER_NAME "VisionLCMGL"

#define PARAM_RENDER_IN "Show"

#define MAX_GROUND_PROJECTION_DISTANCE 120
#define MAX_GROUND_PROJECTION_DISTANCE_SQ (MAX_GROUND_PROJECTION_DISTANCE*MAX_GROUND_PROJECTION_DISTANCE)

#define SLIDER_SCALE_NAME "Image Scale"


typedef struct
{
    GPtrArray *backbuffer;
    GPtrArray *frontbuffer;
    int enabled;
} lcmgl_channel_t;


typedef struct _RendererVisionLcmgl RendererVisionLcmgl;

struct _RendererVisionLcmgl {
    BotRenderer renderer;

    BotFrames *frames;

    BotGtkParamWidget *pw;

    GHashTable *cam_handlers;
    BotParam *param;

    lcm_t *lcm;
    BotViewer *viewer;

    GHashTable *channels;
};

typedef struct _ImageVertex ImageVertex;
struct _ImageVertex{
    float tx;
    float ty;
    float vx;
    float vy;
    float vz;
    int ground_candidate;
};

typedef struct _renderer_vision_lcmgl {
    char *channel;
    BotGtkGlDrawingArea *gl_area;
    bot_core_image_t *last_image;
    BotGlTexture *texture;
    RendererVisionLcmgl *renderer;
    BotGtkParamWidget *pw;
    GtkWidget *expander;
    BotCamTrans *camtrans;
    //GtkAspectFrame *aspect_frame;
    uint8_t *uncompresed_buffer;
    int uncompressed_buffer_size;
    int width, height;
    int is_uploaded;

    //    GLuint vbo;
    GLuint at_camera_dl;
    int img_nvertices;
    ImageVertex *vertices;
    int *vert_indices;
    int n_vert_indices;

    point3d_t *pixel_rays_local;
    point2d_t *ground_projections_local;

    int msg_received;
    int render_place;
    int expanded;

} renderer_vision_lcmgl_t;

enum {
    RENDER_IN_WIDGET,
    RENDER_IN_TOP_RIGHT,
    RENDER_IN_TOP_CENTER,
    RENDER_IN_TOP_LEFT,
    RENDER_IN_BOTTOM_RIGHT,
    RENDER_IN_BOTTOM_CENTER,
    RENDER_IN_BOTTOM_LEFT,
    RENDER_AT_CAMERA,
    RENDER_ON_GROUND,
};

static inline void 
_check_gl_errors (const char *label) {
    GLenum errCode = glGetError ();
    const GLubyte *errStr;
    
    while (errCode != GL_NO_ERROR) {
        errStr = gluErrorString(errCode);
        fprintf (stderr, "OpenGL Error (%s)\n", label);
        fprintf (stderr, "%s", (char*)errStr);
        fprintf (stderr, "\n");
        errCode = glGetError ();
    }
}

static void
renderer_vision_lcmgl_destroy (renderer_vision_lcmgl_t *cr)
{
    if (cr->last_image) {
        bot_core_image_t_destroy (cr->last_image);
    }
    if (cr->camtrans) {
        bot_camtrans_destroy (cr->camtrans);
    }
    // TODO cleanly release display list
    free(cr->vertices);
    free(cr->vert_indices);
    free(cr->ground_projections_local);
    free(cr->pixel_rays_local);

    // TODO
    //    if (cr->texture) {
    //        glutil_texture_free (cr->texture);
    //    }
    if (cr->uncompresed_buffer) {
        free (cr->uncompresed_buffer);
        cr->uncompresed_buffer = NULL;
    }
    free (cr->channel);
    // specifically do not delete the gl_area
    free (cr);

    // TODO: need to free lcmgl channels
}

static void on_image (const lcm_recv_buf_t *rbuf, const char *channel, 
                      const bot_core_image_t *msg, void *user_data);
static void on_lcmgl_data (const lcm_recv_buf_t *rbuf, const char *channel, 
                           const bot_lcmgl_data_t *_msg, void *user_data);
static void renderer_vision_lcmgl_draw (renderer_vision_lcmgl_t *cr);
static int renderer_vision_lcmgl_prepare_texture(renderer_vision_lcmgl_t *cr);

static void
_draw_thumbs_at_cameras(RendererVisionLcmgl *self)
{
    // transform into body frame
    GList *crlist = bot_g_hash_table_get_vals (self->cam_handlers);
    for (GList *criter = crlist; criter; criter=criter->next) {
        renderer_vision_lcmgl_t *cr = (renderer_vision_lcmgl_t*) criter->data;

        if (!cr->last_image) 
            continue;

        int rmode = bot_gtk_param_widget_get_enum (cr->pw, PARAM_RENDER_IN);
        if (rmode != RENDER_AT_CAMERA && rmode != RENDER_ON_GROUND) 
            continue;

        if (0 != renderer_vision_lcmgl_prepare_texture(cr))
            continue;

        // load the texture
        GLuint texname = bot_gl_texture_get_texname(cr->texture);
        GLenum textarget = bot_gl_texture_get_target(cr->texture);

        glEnable(textarget);
        glBindTexture(textarget, texname);

        float texcoord_scale_x = 1;
        float texcoord_scale_y = 1;
        if(textarget == GL_TEXTURE_2D) {
            texcoord_scale_x = 1 / (float) cr->width;
            texcoord_scale_y = 1 / (float) cr->height;
        }

        // disable the depth mask while drawing these images
        glDepthMask(GL_FALSE);
        
        glColor3f (1, 1, 1);

        const char *cam_name;
        if (cr->camtrans)
            cam_name = bot_camtrans_get_name(cr->camtrans);
        else {
            fprintf (stderr, "cr->camtrans is NULL!!\n");
            return;
        }

        // draw
        if (rmode == RENDER_AT_CAMERA) {
            BotTrans cam_to_local;
            bot_frames_get_trans (self->frames, cam_name, "local", &cam_to_local);

            double c2l_mat[16], c2l_mat_gl[16];
            bot_trans_get_mat_4x4(&cam_to_local, c2l_mat);
            bot_matrix_transpose_4x4d(c2l_mat, c2l_mat_gl);

            // transform into body frame
            glPushMatrix();
            glMultMatrixd(c2l_mat_gl);

            if (cr->at_camera_dl) {
                glCallList (cr->at_camera_dl);
            } else {
                // compile a display list if needed
                cr->at_camera_dl = glGenLists (1);
                glNewList (cr->at_camera_dl, GL_COMPILE);

                glBegin(GL_QUADS);
                for (int i=0; i<cr->n_vert_indices; i++) {
                    ImageVertex *v = &cr->vertices[cr->vert_indices[i]];
                    glTexCoord2f(v->tx, v->ty);
                    glVertex3f(v->vx, v->vy, v->vz);
                }
                glEnd();

                glEndList ();
            }

            glPopMatrix();
        } else if (rmode == RENDER_ON_GROUND) {
            BotTrans cam_to_local;
            bot_frames_get_trans_with_utime(self->frames, cam_name, "local", 
                                            cr->last_image->utime, &cam_to_local);

            double vehicle_pos[3];
            double vehicle_pos_body[] = {0, 0, 0};
            bot_frames_transform_vec (self->frames, "body", "local", vehicle_pos_body, vehicle_pos);

            // project image onto the ground plane
            for (int i=0; i<cr->img_nvertices; i++) {
                ImageVertex *v = &cr->vertices[i];
                if (!v->ground_candidate)
                    continue;

                cr->pixel_rays_local[i].x = v->vx; 
                cr->pixel_rays_local[i].y = v->vy; 
                cr->pixel_rays_local[i].z = v->vz; 
                double v_cam[3] = { v->vx, v->vy, v->vz };
                
                bot_trans_rotate_vec(&cam_to_local, v_cam,
                                     point3d_as_array(&cr->pixel_rays_local[i]));

                if(0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec),
                                                      &cr->pixel_rays_local[i], vehicle_pos[2],
                                                      //                if(0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_pos_body),
                                                      //                            &cr->pixel_rays_local[i], 0,
                                                      &cr->ground_projections_local[i])) {
                    cr->ground_projections_local[i].x = NAN;
                    cr->ground_projections_local[i].y = NAN;
                    continue;
                }

                double dist_sq = 
                    geom_point_point_distance_squared_2d (&cr->ground_projections_local[i],
                                                          POINT2D(cam_to_local.trans_vec));
                //                            POINT2D(cam_pos_body));

                if(dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
                    cr->ground_projections_local[i].x = NAN;
                    cr->ground_projections_local[i].y = NAN;
                }
            }

            // draw the texels that have a reasonable chance of actually being on the ground plane
            glBegin(GL_QUADS);
            for (int i=0; i<cr->n_vert_indices; i+=4) {
                ImageVertex *v0 = &cr->vertices[cr->vert_indices[i+0]];
                ImageVertex *v1 = &cr->vertices[cr->vert_indices[i+1]];
                ImageVertex *v2 = &cr->vertices[cr->vert_indices[i+2]];
                ImageVertex *v3 = &cr->vertices[cr->vert_indices[i+3]];

                point2d_t *p0 = &cr->ground_projections_local[cr->vert_indices[i+0]];
                point2d_t *p1 = &cr->ground_projections_local[cr->vert_indices[i+1]];
                point2d_t *p2 = &cr->ground_projections_local[cr->vert_indices[i+2]];
                point2d_t *p3 = &cr->ground_projections_local[cr->vert_indices[i+3]];

                if (!(v0->ground_candidate && 
                      v1->ground_candidate &&
                      v2->ground_candidate &&
                      v3->ground_candidate))
                    continue;

                if (isnan(p0->x) || 
                    isnan(p1->x) ||
                    isnan(p2->x) ||
                    isnan(p3->x))
                    continue;

                glTexCoord2f(v0->tx, v0->ty);
                glVertex3f(p0->x, p0->y, vehicle_pos[2]);
                glTexCoord2f(v1->tx, v1->ty);
                glVertex3f(p1->x, p1->y, vehicle_pos[2]);
                glTexCoord2f(v2->tx, v2->ty);
                glVertex3f(p2->x, p2->y, vehicle_pos[2]);
                glTexCoord2f(v3->tx, v3->ty);
                glVertex3f(p3->x, p3->y, vehicle_pos[2]);
            }
            glEnd();
        }
        glDepthMask(GL_TRUE);
        

        glBindTexture(textarget, 0);
        glDisable(textarget);
    }
}

static void
vision_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererVisionLcmgl *self = (RendererVisionLcmgl*) renderer->user;

    _draw_thumbs_at_cameras(self);

    // transform into window coordinates, where <0, 0> is the top left corner
    // of the window and <viewport[2], viewport[3]> is the bottom right corner
    // of the window
    GLint viewport[4];
    glGetIntegerv (GL_VIEWPORT, viewport);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, viewport[2], 0, viewport[3]);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0, viewport[3], 0);
    glScalef(1, -1, 1);

    glEnable(GL_DEPTH_TEST);
    
    double vp_width = viewport[2] - viewport[0];
    double vp_height = viewport[3] - viewport[1];

    GList *crlist = bot_g_hash_table_get_vals (self->cam_handlers);
    for (GList *criter = crlist; criter; criter=criter->next) {
        renderer_vision_lcmgl_t *cr = (renderer_vision_lcmgl_t*) criter->data;

        if (!cr->last_image) continue;
        double aspect = cr->last_image->width / 
            (double) cr->last_image->height;

        float image_scale = 0.333;
        image_scale = bot_gtk_param_widget_get_double (cr->pw, SLIDER_SCALE_NAME);
        double thumb_width, thumb_height;
        if ((vp_width * image_scale) / aspect > vp_height * image_scale) {
            thumb_height = vp_height * image_scale;
            thumb_width = thumb_height * aspect;
        } else {
            thumb_width = vp_width * image_scale;
            thumb_height = thumb_width / aspect;
        }

        int rmode = bot_gtk_param_widget_get_enum (cr->pw, PARAM_RENDER_IN);
        if (rmode == RENDER_IN_WIDGET || 
            rmode == RENDER_AT_CAMERA ||
            rmode == RENDER_ON_GROUND) 
            continue;

        point2d_t p1 = { viewport[0], viewport[1] }; 

        switch (rmode) {
        case RENDER_IN_BOTTOM_RIGHT:
            p1.x = vp_width - thumb_width;
            p1.y = vp_height - thumb_height;
            break;
        case RENDER_IN_BOTTOM_CENTER:
            p1.x = vp_width * 0.5 * (1.0f - image_scale);
            p1.y = vp_height - thumb_height;
            break;
        case RENDER_IN_BOTTOM_LEFT:
            p1.x = 0;
            p1.y = vp_height - thumb_height;
            break;
        case RENDER_IN_TOP_LEFT:
            p1.x = 0;
            p1.y = 0;
            break;
        case RENDER_IN_TOP_CENTER:
            p1.x = vp_width * 0.5 * (1.0f - image_scale);
            p1.y = 0;
            break;
        case RENDER_IN_TOP_RIGHT:
            p1.x = vp_width - thumb_width;
            p1.y = 0;
            break;
        default:
            break;
        }

        glPushMatrix ();
        glTranslatef (p1.x, p1.y, 1);
        glScalef (thumb_width, thumb_height, 1);
        renderer_vision_lcmgl_draw (cr);
        
        glPopMatrix ();
    }
    g_list_free (crlist);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVisionLcmgl *self = user_data;

    GError *gerr = NULL;
    char **keys = g_key_file_get_keys (keyfile, RENDERER_NAME, NULL, &gerr);
    if (gerr) {
        g_error_free (gerr);
        return;
    }
    for (int i=0; keys[i]; i++) {
        char *key = keys[i];
        renderer_vision_lcmgl_t *cr =
            g_hash_table_lookup (self->cam_handlers, key);
        if (!cr) {
            cr = (renderer_vision_lcmgl_t*) calloc (1, sizeof (renderer_vision_lcmgl_t));
            cr->channel = strdup (key);
            cr->renderer = self;
            g_hash_table_replace (self->cam_handlers, cr->channel, cr);
        }
        char *val = g_key_file_get_string (keyfile, RENDERER_NAME, key, NULL);
        cr->render_place = 0;
        cr->expanded = 0;
        if (val) {
            sscanf (val, "%d %d", &cr->render_place, &cr->expanded);
        }
        g_free(val);
    }
    g_strfreev (keys);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVisionLcmgl *self = user_data;
    GList *keys = bot_g_hash_table_get_keys(self->cam_handlers);

    for (GList *kiter=keys; kiter; kiter=kiter->next) {
        char *key = (char*) kiter->data;
        renderer_vision_lcmgl_t *cr = g_hash_table_lookup(self->cam_handlers, key);

        char str[80];
        sprintf (str, "%d %d", cr->render_place, cr->expanded);
        g_key_file_set_string (keyfile, RENDERER_NAME, key, str);
    }
    g_list_free (keys);
}

static void
vision_free (BotRenderer *renderer) 
{
    RendererVisionLcmgl *self = (RendererVisionLcmgl*) renderer;

    g_hash_table_destroy (self->cam_handlers);
    free (renderer);
}


static int
renderer_vision_lcmgl_prepare_texture(renderer_vision_lcmgl_t *cr)
{
    if (! cr->renderer->renderer.enabled) return -1;
    if (! cr->last_image) return -1;

    // create the texture object if necessary
    if (! cr->texture) {
        cr->texture = bot_gl_texture_new (cr->last_image->width, 
                                          cr->last_image->height, 
                                          cr->last_image->width * 3 * cr->last_image->height);
    }

    // upload the texture to the graphics card if necessary
    if (!cr->is_uploaded) {
        int stride = 0;
        GLenum gl_format;
        uint8_t *tex_src = NULL;

        if (cr->last_image->pixelformat == 0 || 
            cr->last_image->pixelformat == PIXEL_FORMAT_GRAY ||
            cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_BGGR ||
            cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_RGGB ||
            cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_GRBG ||
            cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_GBRG) {

            stride = cr->last_image->width;
            gl_format = GL_LUMINANCE;
            tex_src = cr->last_image->data;
        } else if (cr->last_image->pixelformat == PIXEL_FORMAT_MJPEG) {
            bot_core_image_t * msg = cr->last_image;

            // might need to JPEG decompress...
            stride = cr->last_image->width * 3;
            int buf_size = msg->height * stride;
            if (cr->uncompressed_buffer_size < buf_size) {
                cr->uncompresed_buffer = 
                    realloc (cr->uncompresed_buffer, buf_size);
                cr->uncompressed_buffer_size = buf_size;
            }
            jpeg_decompress_8u_rgb (msg->data, msg->size, 
                                    cr->uncompresed_buffer, msg->width, msg->height, stride);

            gl_format = GL_RGB;
            tex_src = cr->uncompresed_buffer;
        } else {
            return -1;
        }
        bot_gl_texture_upload (cr->texture, gl_format, GL_UNSIGNED_BYTE,
                               stride, tex_src);
        cr->is_uploaded = 1;
    }

    if (!cr->vertices) {
        int xstep = 4;
        int ystep = 12;
        int ncols = cr->width / xstep + 1;
        int nrows = cr->height / ystep + 1;
        cr->img_nvertices = ncols * nrows;
        int img_data_size = cr->img_nvertices * sizeof(ImageVertex);
        cr->vertices = (ImageVertex*) malloc(img_data_size);

        cr->n_vert_indices = (ncols-1) * (nrows-1) * 4;

        //cr->n_vert_indices = cr->width / xstep * cr->height / ystep * 4; 
        cr->vert_indices = (int*) malloc(cr->n_vert_indices * sizeof(int));
        int vi_count = 0;

        // allocate workspace for projecting the image onto the ground plane
        cr->pixel_rays_local = (point3d_t*) malloc(cr->img_nvertices * sizeof(point3d_t));
        cr->ground_projections_local = (point2d_t*) malloc(cr->img_nvertices * sizeof(point2d_t));

        // precompute undistorted coordinates
        GLenum textarget = bot_gl_texture_get_target(cr->texture);

        float texcoord_scale_x = 1;
        float texcoord_scale_y = 1;
        if(textarget == GL_TEXTURE_2D) {
            texcoord_scale_x = 1 / (float) cr->width;
            texcoord_scale_y = 1 / (float) cr->height;
        }

        ImageVertex *v = cr->vertices;

        const char *cam_name = bot_camtrans_get_name(cr->camtrans);
        BotTrans cam_to_body;
        bot_frames_get_trans(cr->renderer->frames, cam_name, "body", &cam_to_body);
        double *cam_pos_body = cam_to_body.trans_vec;

        for (int row=0; row<nrows; row++) {
            int y = row * ystep;
            for (int col=0; col<ncols; col++) {
                int x = col * xstep;
                double ray_cam[3], ray_body[3];

                bot_camtrans_unproject_pixel (cr->camtrans, x, y, ray_cam);

                bot_vector_normalize_3d(ray_cam);
                bot_trans_rotate_vec(&cam_to_body, ray_cam, ray_body);

                v->tx = x * texcoord_scale_x;
                v->ty = y * texcoord_scale_y;
                v->vx = ray_cam[0];
                v->vy = ray_cam[1];
                v->vz = ray_cam[2];

                v->ground_candidate = 0;
                if (ray_body[2] < 0) {
                    point2d_t isect_pt;
                    if (0 == geom_ray_z_plane_intersect_3d(POINT3D(cam_pos_body), 
                                                           POINT3D(ray_body), 0, &isect_pt)) {

                        double dist = geom_point_point_distance_squared_2d(&isect_pt, 
                                                                           POINT2D(cam_pos_body));

                        if (dist < MAX_GROUND_PROJECTION_DISTANCE_SQ) {
                            v->ground_candidate = 1;
                        }
                    }
                }

                v++;

                if (row < nrows - 1 && col < ncols - 1) {
                    cr->vert_indices[vi_count + 0] = row * ncols + col;
                    cr->vert_indices[vi_count + 1] = row * ncols + col + 1;
                    cr->vert_indices[vi_count + 2] = (row + 1) * ncols + col + 1;
                    cr->vert_indices[vi_count + 3] = (row + 1) * ncols + col;

                    for (int i=0; i<4; i++)
                        assert(cr->vert_indices[vi_count + i] < cr->img_nvertices);

                    vi_count += 4;
                }
                assert(vi_count <= cr->n_vert_indices);
            }
        }
        assert(vi_count == cr->n_vert_indices);
    }

    return 0;
}

static void
renderer_vision_lcmgl_draw (renderer_vision_lcmgl_t *cr)
{
    glDisable(GL_DEPTH_TEST);

    // render the image
    renderer_vision_lcmgl_prepare_texture(cr);
    glColor3f(1,1,1);
    bot_gl_texture_draw (cr->texture);

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(cr->renderer->channels);

    for (GList *kiter = keys; kiter; kiter=kiter->next) {
        lcmgl_channel_t *chan = g_hash_table_lookup(cr->renderer->channels, 
                                                    kiter->data);
        glPushMatrix();
        glPushAttrib(GL_ENABLE_BIT);

        glScalef (1.0f/cr->last_image->width, 1.0f/cr->last_image->height, 1);

        if (chan->enabled) {
            // iterate over all the messages received for this channel
            for (int i = 0; i < chan->frontbuffer->len; i++) {
                bot_lcmgl_data_t *data = 
                    g_ptr_array_index(chan->frontbuffer, i);
                
                bot_lcmgl_decode(data->data, data->datalen);
            }
        }
        glPopAttrib ();
        glPopMatrix();
    }
    g_list_free (keys);
}

static gboolean
on_gl_area_expose (GtkWidget * widget, GdkEventExpose * event, void* user_data)
{
    renderer_vision_lcmgl_t *cr = (renderer_vision_lcmgl_t*) user_data;

    bot_gtk_gl_drawing_area_set_context (cr->gl_area);

    glClearColor (0.0, 0.0, 0.0, 1.0);
    glClear (GL_COLOR_BUFFER_BIT);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();

    glOrtho (0, 1, 1, 0, -1, 1);
    glMatrixMode (GL_MODELVIEW);

    renderer_vision_lcmgl_draw (cr);

    bot_gtk_gl_drawing_area_swap_buffers (cr->gl_area);

    bot_gtk_gl_drawing_area_set_context(cr->renderer->viewer->gl_area);
    return TRUE;
}

static void
on_gl_area_size (GtkWidget * widget, GtkAllocation * alloc,
                 renderer_vision_lcmgl_t * cr)
{
    gtk_widget_set_size_request (widget, -1,
                                 alloc->width * cr->height / cr->width);
}

static void
on_expander_expanded (GtkExpander *expander, GParamSpec *pspec, void *user_data)
{
    renderer_vision_lcmgl_t *cr = user_data;
    cr->expanded = gtk_expander_get_expanded (expander);
}

static void
on_renderer_vision_lcmgl_param_widget_changed (BotGtkParamWidget *pw,
                                               const char *param, void *user_data)
{
    renderer_vision_lcmgl_t *cr = (renderer_vision_lcmgl_t*) user_data;

    // delete the old texture object if it exists.  make sure that we've
    // selected the correct OpenGL context
    if (cr->texture) {
        if (cr->render_place == RENDER_IN_WIDGET) {
            bot_gtk_gl_drawing_area_set_context (cr->gl_area);
        } else {
            bot_gtk_gl_drawing_area_set_context (cr->renderer->viewer->gl_area);
        }

        bot_gl_texture_free (cr->texture);
        cr->texture = NULL;
    }

    cr->render_place = bot_gtk_param_widget_get_enum (pw, PARAM_RENDER_IN);
    if (cr->render_place == RENDER_IN_WIDGET) {
        gtk_widget_show (GTK_WIDGET (cr->gl_area));
    } else {
        gtk_widget_hide (GTK_WIDGET (cr->gl_area));
    }

    cr->is_uploaded = 0;

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(cr->renderer->channels);

    for (GList *kiter=keys; kiter; kiter=kiter->next) {
        lcmgl_channel_t *chan = g_hash_table_lookup(cr->renderer->channels, 
                                                    kiter->data);
        
        chan->enabled = bot_gtk_param_widget_get_bool (pw, kiter->data);
    }
    g_list_free (keys);

    bot_viewer_request_redraw(cr->renderer->viewer);
}

size_t strnlen(const char *s, size_t len)
{
    size_t i;
    for(i=0; i<len && *(s+i); i++);
    return i;
}

/*char* strndup (char const *s, size_t n)
{
    size_t len = strnlen (s, n);
    char *new = malloc (len + 1);

    if (new == NULL)
        return NULL;

    new[len] = '\0';
    return memcpy (new, s, len);
    } */

static void on_lcmgl_data (const lcm_recv_buf_t *rbuf, const char *channel, 
                           const bot_lcmgl_data_t *_msg, void *user_data )
{
    RendererVisionLcmgl *self = (RendererVisionLcmgl*) user_data;
    
    // parse out which camera from channel string
    
    regex_t preg;
    if (0 != regcomp (&preg, LCMGL_VISION_CHANNEL_MATCH_REGEX,
                      REG_EXTENDED)) {
        printf("%s:%d Error calling regcomp on %s\n",
               __FILE__, __LINE__, LCMGL_VISION_CHANNEL_MATCH_REGEX);
        return;
    }
    
    regmatch_t match[3];
    if (0 != regexec (&preg, channel, 3, match, 0)) {
        regfree (&preg);
        return;
    }
    
    char *which_cam = strndup (&(channel[match[1].rm_so]),
                               match[1].rm_eo - match[1].rm_so);
    char *which_lcmgl = strndup (&(channel[match[2].rm_so]),
                                 match[2].rm_eo - match[2].rm_so);

    regfree (&preg);

    renderer_vision_lcmgl_t *cr = g_hash_table_lookup (self->cam_handlers,
                                                       which_cam);
    if (NULL == cr) {
        printf ("Warning: cannot find renderer for channel %s\n", which_cam);
        goto cleanup;
    }

    lcmgl_channel_t *chan = g_hash_table_lookup(cr->renderer->channels,
                                                which_lcmgl);
    if (!chan) {
        chan = (lcmgl_channel_t*) calloc(1, sizeof(lcmgl_channel_t));
        chan->enabled=1;
        //chan->backbuffer = g_ptr_array_new();
        chan->frontbuffer = g_ptr_array_new();
        g_hash_table_insert(cr->renderer->channels, strdup(which_lcmgl), chan);
        bot_gtk_param_widget_add_booleans (cr->pw, 
                                           0, strdup(which_lcmgl), 1, NULL);
    }

#if 0
    int current_scene = -1;
    if (chan->backbuffer->len > 0) {
        bot_lcmgl_data_t *ld = g_ptr_array_index(chan->backbuffer, 0);
        current_scene = ld->scene;
    }

    // new scene?
    if (current_scene != _msg->scene) {

        // free objects in foreground buffer
        for (int i = 0; i < chan->frontbuffer->len; i++)
            bot_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
        g_ptr_array_set_size(chan->frontbuffer, 0);
        
        // swap front and back buffers
        GPtrArray *tmp = chan->backbuffer;
        chan->backbuffer = chan->frontbuffer;
        chan->frontbuffer = tmp;
        
        bot_viewer_request_redraw( self->renderer->viewer );
    }
#endif

    for (int i = 0; i < chan->frontbuffer->len; i++)
        bot_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
    g_ptr_array_set_size (chan->frontbuffer, 0);
    g_ptr_array_add(chan->frontbuffer, bot_lcmgl_data_t_copy(_msg));
    bot_viewer_request_redraw( cr->renderer->viewer );

 cleanup:
    free (which_cam);
    free (which_lcmgl);
    return;
}

static void 
on_image (const lcm_recv_buf_t *rbuf, const char *channel, 
          const bot_core_image_t *msg, void *user_data)
{
    RendererVisionLcmgl *self = (RendererVisionLcmgl*) user_data;

    renderer_vision_lcmgl_t *cr = g_hash_table_lookup (self->cam_handlers, channel);
    if (!cr) {
        cr = (renderer_vision_lcmgl_t*) calloc (1, sizeof (renderer_vision_lcmgl_t));
        cr->renderer = self;
        cr->render_place = 0;
        cr->channel = strdup (channel);
        cr->at_camera_dl = 0;
        g_hash_table_replace (self->cam_handlers, cr->channel, cr);
    }

    if (! cr->msg_received) {
        cr->gl_area = BOT_GTK_GL_DRAWING_AREA (bot_gtk_gl_drawing_area_new (FALSE));

        cr->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
        bot_gtk_param_widget_add_enum (cr->pw, PARAM_RENDER_IN, 
                                       0,
                                       cr->render_place,
                                       "Here", RENDER_IN_WIDGET,
                                       "Top Left", RENDER_IN_TOP_LEFT,
                                       "Top Cent.", RENDER_IN_TOP_CENTER,
                                       "Top Right", RENDER_IN_TOP_RIGHT,
                                       "Bot. Left", RENDER_IN_BOTTOM_LEFT,
                                       "Bot. Cent.", RENDER_IN_BOTTOM_CENTER,
                                       "Bot. Right", RENDER_IN_BOTTOM_RIGHT,
                                       "At Camera", RENDER_AT_CAMERA,
                                       "Ground", RENDER_ON_GROUND,
                                       NULL);
        bot_gtk_param_widget_add_double (cr->pw, SLIDER_SCALE_NAME,
                                         BOT_GTK_PARAM_WIDGET_SLIDER,
                                         0.1, 1.0, 0.01, 0.333);

        cr->expander = gtk_expander_new (channel);
        gtk_box_pack_start (GTK_BOX (self->renderer.widget),
                            cr->expander, TRUE, TRUE, 0);
        GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
        gtk_container_add (GTK_CONTAINER (cr->expander), vbox);

        gtk_box_pack_start (GTK_BOX (vbox), 
                            GTK_WIDGET (cr->pw), TRUE, TRUE, 0);
        gtk_box_pack_start (GTK_BOX (vbox), 
                            GTK_WIDGET (cr->gl_area), TRUE, TRUE, 0);

        g_signal_connect (G_OBJECT (cr->gl_area), "size-allocate",
                          G_CALLBACK (on_gl_area_size), cr);
        cr->width = msg->width;
        cr->height = msg->height;

        gtk_widget_show_all (GTK_WIDGET (cr->expander));
        gtk_expander_set_expanded (GTK_EXPANDER (cr->expander), cr->expanded);

        if (cr->render_place == RENDER_IN_WIDGET) {
            gtk_widget_show (GTK_WIDGET (cr->gl_area));
        } else {
            gtk_widget_hide (GTK_WIDGET (cr->gl_area));
        }

        g_signal_connect (G_OBJECT (cr->pw), "changed",
                          G_CALLBACK (on_renderer_vision_lcmgl_param_widget_changed), cr);
        g_signal_connect (G_OBJECT (cr->gl_area), "expose-event", 
                          G_CALLBACK (on_gl_area_expose), cr);

        g_signal_connect (G_OBJECT (cr->expander), "notify::expanded",
                          G_CALLBACK (on_expander_expanded), cr);

        cr->texture = NULL;
        cr->last_image = NULL;
        cr->renderer = self;
        cr->uncompressed_buffer_size = msg->width * msg->height * 3;
        cr->uncompresed_buffer = 
            (uint8_t*) malloc (cr->uncompressed_buffer_size);

        char cam_name[1024];
        char **cam_names = bot_param_get_all_camera_names (self->param);
        int found = 0;
        if (cam_names) {
            for (int i=0; cam_names[i]; i++) {
                char key[1024];
                char *cam_vision_channel = NULL;
                snprintf (key, sizeof (key), "cameras.%s.streams.vision.channel", cam_names[i]);
                if (0 == bot_param_get_str (self->param, key, &cam_vision_channel)) {
                    if (!strcmp (channel, cam_vision_channel)) {
                        strcpy (cam_name, cam_names[i]);
                        found = 1;
                        break;
                    }
                }
            }
        }

        if (found == 1) {
            cr->camtrans = bot_param_get_new_camtrans (self->param, cam_name);

            double xscale = cr->width / bot_camtrans_get_image_width(cr->camtrans);
            double yscale = cr->width / bot_camtrans_get_image_width(cr->camtrans);
            assert(fabs(xscale - yscale) < 1e-6);
            bot_camtrans_scale_image(cr->camtrans, xscale);
        } else {
            printf("%s:%d couldn't find camera calibration for %s\n", 
                   __FILE__, __LINE__, channel);
            cr->camtrans = NULL;
        }

        cr->msg_received = 1;
    }

    if (cr->last_image) {
        bot_core_image_t_destroy (cr->last_image);
    }
    cr->last_image = bot_core_image_t_copy (msg);
    cr->is_uploaded = 0;

    switch (bot_gtk_param_widget_get_enum (cr->pw, PARAM_RENDER_IN)) {
    case RENDER_IN_WIDGET:
        if (gtk_expander_get_expanded (GTK_EXPANDER (cr->expander)))
            bot_gtk_gl_drawing_area_invalidate (cr->gl_area);
    default:
        bot_viewer_request_redraw (self->viewer);
        break;
    }
}

static BotRenderer *renderer_vision_lcmgl_new (BotViewer *viewer, lcm_t *lcm, BotParam *param)
{

    /* TODO: need this somewhere?
       g_signal_connect (G_OBJECT (self->pw), "changed", 
       G_CALLBACK (on_param_widget_changed), self);

    */

    RendererVisionLcmgl *self = 
        (RendererVisionLcmgl*) calloc (1, sizeof (RendererVisionLcmgl));
    self->viewer = viewer;
    self->renderer.draw = vision_draw;
    self->renderer.destroy = vision_free;
    self->renderer.name = "Vision LCM GL";
    self->renderer.user = self;
    self->renderer.enabled = 1;

    self->lcm = lcm;
    self->param = param;
    self->frames = bot_frames_get_global (lcm, param);

    self->renderer.widget = gtk_vbox_new (FALSE, 0);
    gtk_widget_show (self->renderer.widget);

    self->cam_handlers = g_hash_table_new_full (g_str_hash, g_str_equal,
                                                NULL, (GDestroyNotify) renderer_vision_lcmgl_destroy);
    
    self->channels = g_hash_table_new(g_str_hash, g_str_equal);

    char **cam_names = bot_param_get_all_camera_names (self->param);
    if (cam_names) {
        for (int i=0; cam_names[i]; i++) {
            char *channel;
            char key[1024];
            snprintf (key, sizeof (key), "cameras.%s.streams.vision.channel", cam_names[i]);
            if (0 == bot_param_get_str (self->param, key, &channel)) {
                bot_core_image_t_subscribe (self->lcm, channel, on_image, self);
            }
            else
                fprintf (stdout, "renderer_vision_lcmgl: No LCM channel found for %s.stream.vision.\n",
                         cam_names[i]);
        }
    }
    else
        fprintf (stdout, "renderer_vision_lcmgl: No cameras found via param server!\n");

    g_strfreev (cam_names);

    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    gtk_box_pack_start (GTK_BOX (self->renderer.widget), GTK_WIDGET (self->pw), 
                        TRUE, TRUE, 0);

    bot_lcmgl_data_t_subscribe(self->lcm, LCMGL_VISION_CHANNEL_REGEX,
                               on_lcmgl_data, self);

    gtk_widget_show (GTK_WIDGET (self->pw));

    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    return &self->renderer;
}

void setup_renderer_vision_lcmgl (BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam *param)
{
    bot_viewer_add_renderer(viewer, renderer_vision_lcmgl_new(viewer, lcm, param),
                        render_priority);
}
