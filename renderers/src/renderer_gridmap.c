/*
 * random rendering code.  Mostly rendering of gridmaps
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dirent.h>

#ifdef __APPLE__
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_vis/gtk_util.h>
#include <bot_vis/gl_util.h>
#include <bot_vis/viewer.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <hr_common/gridmap.h>

#include <lcmtypes/erlcm_rect_list_t.h>
#include <lcmtypes/erlcm_obstacle_list_t.h>

#include "tile_set.h"


#define RENDERER_NAME "GridMap"

#define MAX_TILE_AGE 1

#define PARAM_RECTS "Rects"
#define PARAM_RECTS_HEIGHT "Rect height"
#define PARAM_RECTS_POS "Rects pos display"
#define PARAM_OBSTACLE_COST "Obstacle cost"
#define PARAM_NAME_OBST_OPACITY "Obst opacity"


typedef struct _RendererGridMap {
    BotRenderer        renderer;
    BotParam           *param;
    lcm_t              *lcm;
    GHashTable         *channels_hashtable;
    GPtrArray          *channels;
    BotViewer          *viewer;
    BotGtkParamWidget  *pw;
    GHashTable         *tileset_hashtable;
    BotFrames          *frames;
    erlcm_rect_list_t  *rects;
    erlcm_rect_list_t  *map_rects;
} RendererGridMap;


static int 
frames_vehicle_pos_local(BotFrames *frames, double pos[3])
{
    double pos_body[3] = { 0, 0, 0 };
    return bot_frames_transform_vec(frames, "body", "local", pos_body, pos);
}


static gboolean 
tileset_remove_helper(gpointer k, gpointer v, gpointer user)
{
    tile_set_destroy(v);
    return TRUE;
}

static void 
my_free( BotRenderer *renderer )
{
    RendererGridMap *self = (RendererGridMap*) renderer->user;

    g_hash_table_foreach_remove(self->tileset_hashtable, tileset_remove_helper, NULL);
    g_hash_table_destroy ( self->tileset_hashtable );

    g_ptr_array_free(self->channels, TRUE);
    free( self );
}

static void
on_obstacles (const lcm_recv_buf_t * rbuf, const char * channel,
              const erlcm_obstacle_list_t * msg, void * user)
    
{
    RendererGridMap *self = (RendererGridMap*) user;
    
    if (self->rects)
        erlcm_rect_list_t_destroy(self->rects);
    
    self->rects = erlcm_rect_list_t_copy(&msg->rects);
    
    return;
}

static void
on_map_rects (const lcm_recv_buf_t * rbuf, const char * channel,
              const erlcm_rect_list_t * msg, void * user)
    
{
    RendererGridMap *self = (RendererGridMap*) user;
    
    if (self->map_rects)
        erlcm_rect_list_t_destroy(self->map_rects);
    
    self->map_rects = erlcm_rect_list_t_copy(msg);
    
    return;
}

static void
on_gridmap_tile(const lcm_recv_buf_t * rbuf, const char * channel,
                const erlcm_gridmap_tile_t * tile, void * user)
{
    RendererGridMap *self = (RendererGridMap*) user;

    if (!strcmp(channel, "OBSTACLE_MAP") && (!self->viewer ||
                                             bot_gtk_param_widget_get_bool(self->pw, PARAM_OBSTACLE_COST))) {
        tile_set_process_new_tile (g_hash_table_lookup (self->tileset_hashtable, channel), tile);
    }

    if (self->viewer)
        bot_viewer_request_redraw(self->viewer);

    return;
}

static void 
draw_rects(RendererGridMap *self)
{
    if (!(self->rects) && !(self->map_rects))
        return;

    double h = !self->viewer ? 1.0 : bot_gtk_param_widget_get_double(self->pw, PARAM_RECTS_HEIGHT);
    double alpha = !self->viewer ? 1.0 : bot_gtk_param_widget_get_double(self->pw, PARAM_NAME_OBST_OPACITY);
  

    // Render the perceived rects
    if (self->rects) {
        if (h > 0) {
            glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
            //glEnable (GL_LIGHTING);
            glDisable (GL_LIGHTING);
            glEnable (GL_DEPTH_TEST);
            glShadeModel (GL_SMOOTH);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glEnable (GL_COLOR_MATERIAL);
            float c[] = { 0.5, 0.5, 0.5, alpha };
            glColor4fv (c);
            //glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, c);
        } else {
            glColor4f (.3, .3, .3, alpha);
        }
        
        erlcm_rect_t *rects = self->rects->rects;
        for (int i = 0; i < self->rects->num_rects; i++) {
            double x0 = self->rects->xy[0] + rects[i].dxy[0];
            double y0 = self->rects->xy[1] + rects[i].dxy[1];
            double sx = rects[i].size[0] / 2;
            double sy = rects[i].size[1] / 2;
            
            double MIN_RECT_SIZE = 0.1;
            sx = fmax(sx, MIN_RECT_SIZE);
            sy = fmax(sy, MIN_RECT_SIZE);
            
            glPushMatrix();
            
            glTranslated(x0, y0, h/2);
            glRotatef(bot_to_degrees(rects[i].theta), 0, 0, 1);
            if (h == 0) {
                glBegin(GL_QUADS);
                glVertex2f( - sx, - sy);
                glVertex2f( - sx, + sy);
                glVertex2f( + sx, + sy);
                glVertex2f( + sx, - sy);
                glEnd();
            } else {
                glScalef (sx, sy, h/2);
                bot_gl_draw_cube ();
                glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_CURRENT_BIT);
                glColor4f (0.5, 0.5, 0.5, alpha);
                bot_gl_draw_cube_frame ();
                glPopAttrib();
            }
            
            glPopMatrix();
        }
        
        if (h > 0) {
            glPopAttrib();
        }
    }
    
    // Render rects contained in the map (expressed in global coordinates)
    if (self->map_rects) {
        if (h > 0) {
            glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
            //glEnable (GL_LIGHTING);
            glDisable (GL_LIGHTING);
            glEnable (GL_DEPTH_TEST);
            glShadeModel (GL_SMOOTH);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glEnable (GL_COLOR_MATERIAL);
            float c[] = { 0.58, 0.44, 0.86, alpha };
            glColor4fv (c);
        } else {
            glColor4f (.35, .35, 35, alpha);
        }
        
        erlcm_rect_t *rects = self->map_rects->rects;
        for (int i = 0; i < self->map_rects->num_rects; i++) {
            double pos_local[3], rpy_local[3], quat_local[4];
            double pos_global[3] = {self->map_rects->xy[0] + rects[i].dxy[0], 
                                    self->map_rects->xy[1] + rects[i].dxy[1], 0};
            double size_global[3] = {rects[i].size[0]/2, rects[i].size[1]/2, 0};
            double rpy_global[3] = {0, 0, rects[i].theta};
            double quat_global[4];

            // Determine the position and size in the local frame
            bot_frames_transform_vec (self->frames, "global", "local", pos_global, pos_local);

            // Determine the orientation
            BotTrans global_to_local;
            bot_roll_pitch_yaw_to_quat (rpy_global, quat_global);
            bot_frames_get_trans (self->frames, "global", "local", &global_to_local);
            bot_quat_mult (quat_local, global_to_local.rot_quat, quat_global);
            bot_quat_to_roll_pitch_yaw (quat_local, rpy_local);

            double MIN_RECT_SIZE = 0.1;
            double sx = fmax(size_global[0], MIN_RECT_SIZE);
            double sy = fmax(size_global[1], MIN_RECT_SIZE);
            
            glPushMatrix();
            
            double x0 = pos_local[0];
            double y0 = pos_local[1];
            double theta = rpy_local[2];

            glTranslated(x0, y0, h/2);
            glRotatef(bot_to_degrees(theta), 0, 0, 1);
            if (h == 0) {
                glBegin(GL_QUADS);
                glVertex2f( - sx, - sy);
                glVertex2f( - sx, + sy);
                glVertex2f( + sx, + sy);
                glVertex2f( + sx, - sy);
                glEnd();
            } else {
                glScalef (2*sx, 2*sy, h);
                bot_gl_draw_cube ();
                //glutSolidCube(1);
                glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_CURRENT_BIT);
                glColor4f (1, 1, 1, alpha);
                bot_gl_draw_cube_frame ();
                glPopAttrib();
            }
            
            glPopMatrix();
        }
        
        if (h > 0) {
            glPopAttrib();
        }
    }
}

static void 
draw_rects_pos (RendererGridMap *self)
{
    if (!self->rects)
        return;

    BotTrans local_to_body;
    if (!bot_frames_get_trans (self->frames, "local", "body", &local_to_body))
        return;

    double h = !self->viewer ? 1.0 : bot_gtk_param_widget_get_double(self->pw, PARAM_RECTS_HEIGHT);
    double pos_local_to_body[3];
    char buf[256];
    double x0, y0;
    erlcm_rect_t *rects = self->rects->rects;
    for (int i = 0; i < self->rects->num_rects; i++) {
        x0 = self->rects->xy[0] + rects[i].dxy[0];
        y0 = self->rects->xy[1] + rects[i].dxy[1];
        
        const double pos_local[3] = {x0, y0, h/2};
        bot_trans_apply_vec (&local_to_body, pos_local, pos_local_to_body);
        sprintf (buf, "%.2f, %.2f", pos_local_to_body[0], pos_local_to_body[1]);
        glPushAttrib (GL_CURRENT_BIT | GL_ENABLE_BIT);
        glColor4f (1, 1, 1, 1);
        bot_gl_draw_text (pos_local, GLUT_BITMAP_HELVETICA_12, buf,
                BOT_GL_DRAW_TEXT_DROP_SHADOW);
        glPopAttrib ();
    }
}

static void my_draw( BotViewer *viewer, BotRenderer *renderer )
{
    RendererGridMap *self = (RendererGridMap*) renderer->user;

    double pos[3] = { 0, 0, 0 };
    frames_vehicle_pos_local (self->frames, pos);

    glPushAttrib (GL_ENABLE_BIT);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable (GL_COLOR_MATERIAL);

    glPushMatrix ();
    glTranslatef (0, 0, pos[2]);

    if (!viewer || bot_gtk_param_widget_get_bool(self->pw, PARAM_OBSTACLE_COST)) {
        double alpha = !viewer ? 0.0 : bot_gtk_param_widget_get_double(
                self->pw, PARAM_NAME_OBST_OPACITY);
        glColor4d(1, 1, 1, alpha);

        tile_set_draw(g_hash_table_lookup(self->tileset_hashtable, "OBSTACLE_MAP"));
    }
    
    if (!viewer || bot_gtk_param_widget_get_bool(self->pw, PARAM_RECTS))
        draw_rects(self);

    if (!viewer || bot_gtk_param_widget_get_bool (self->pw, PARAM_RECTS_POS))
        draw_rects_pos (self);
    
    glPopMatrix ();

    glPopAttrib();
    //glDisable(GL_COLOR_MATERIAL);

}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererGridMap *self = (RendererGridMap*) user;
    (void) self;

    bot_viewer_request_redraw(self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererGridMap *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererGridMap *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

BotRenderer *renderer_gridmap_new(BotViewer *viewer, lcm_t *_lcm, BotParam * _param)
{
    RendererGridMap *self =
        (RendererGridMap*) calloc(1, sizeof(RendererGridMap));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = my_draw;
    renderer->destroy = my_free;
    renderer->name = RENDERER_NAME;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    self->lcm = _lcm;
    self->param = _param;
    self->frames = bot_frames_get_global (self->lcm, self->param);
    self->channels_hashtable = g_hash_table_new(g_str_hash, g_str_equal);
    self->channels = g_ptr_array_new();
    self->viewer = viewer;
    self->tileset_hashtable = g_hash_table_new(g_str_hash, g_str_equal);

    if (viewer)    {
        bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_RECTS, 1, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_RECTS_POS, 0, NULL);
        bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_OBSTACLE_COST, 1, NULL);
        
        bot_gtk_param_widget_add_double (self->pw,
                                         PARAM_NAME_OBST_OPACITY,
                                         BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.05, 0.3);
        bot_gtk_param_widget_add_double (self->pw,
                                         PARAM_RECTS_HEIGHT,
                                         BOT_GTK_PARAM_WIDGET_SLIDER, 0, 5, 0.5, 0);
        
        g_signal_connect (G_OBJECT (self->pw), "changed",
                          G_CALLBACK (on_param_widget_changed), self);
    }


    ///////////////////////////////////////////////////////
    // obstacle hazard map
#define SIZE 256
    if (1) {
        float r[SIZE];
        float g[SIZE];
        float b[SIZE];
        float a[SIZE];

        for (int i = 0; i < SIZE; i++) {
            double f = ((float) i) / 255;
            double scale = 1.0;
            r[i] = f * scale;
            g[i] = f * scale;
            b[i] = f * scale;
            a[i] = 1;

            if (i & 1) {
                r[i] = 0;
                g[i] = 0;
            }
        }

        r[SIZE-1] = 1.0;
        g[SIZE-1] = .3;
        b[SIZE-1] = .3;
        r[SIZE-2] = 1.0;
        g[SIZE-2] = .3;
        b[SIZE-2] = .3;

        g_hash_table_insert(self->tileset_hashtable, "OBSTACLE_MAP",
                tile_set_create_colormap(r, g, b, a, SIZE));
    }

#undef SIZE

    erlcm_obstacle_list_t_subscribe (self->lcm, "OBSTACLES", on_obstacles, self);
    erlcm_rect_list_t_subscribe (self->lcm, "MAP_SERVER_RECTS", on_map_rects, self);
    erlcm_gridmap_tile_t_subscribe(self->lcm, "OBSTACLE_MAP", on_gridmap_tile, self);

    return renderer;
}

void 
setup_renderer_gridmap(BotViewer *viewer, int render_priority, lcm_t *_lcm, BotParam * param)
{
    BotRenderer *renderer = renderer_gridmap_new(viewer, _lcm, param);
    if (viewer) {
        bot_viewer_add_renderer(viewer, renderer, render_priority);
        
        g_signal_connect (G_OBJECT (viewer), "load-preferences",
                          G_CALLBACK (on_load_preferences), renderer);
        g_signal_connect (G_OBJECT (viewer), "save-preferences",
                          G_CALLBACK (on_save_preferences), renderer);
    }
}

