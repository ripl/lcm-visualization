#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glib.h>
#include <gdk/gdkkeysyms.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <gsl/gsl_blas.h>

#include <bot_core/bot_core.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gtk_util.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include "lcmtypes/hr_lcmtypes.h" 

#define DTOR M_PI/180
#define RTOD 180/M_PI

#define MAX_REFERSH_RATE_USEC 30000 // about 1/30 of a second

#define RENDERER_NAME "PCL Segments"
#define DATA_CIRC_SIZE 10
#define POSE_DATA_CIRC_SIZE 200 

#define PARAM_COLOR_MENU "Color"
#define PARAM_HISTORY_LENGTH "Scan Hist. Len."
#define PARAM_HISTORY_FREQUENCY "Scan Hist. Freq."
#define PARAM_POINT_SIZE "Point Size" 
#define PARAM_POINT_ALPHA "Point Alpha" 

#define DRAW_NORMALS "Normals"
#define DRAW_SEGMENTS "Segments"
#define DRAW_CYLINDERS "Cylinder"
#define DRAW_SEGMENT_PLANES "Segment planes"
#define DRAW_XYZ "xyz"
#define DRAW_XYZRGB "xyz_rgb"




enum {
    COLOR_Z,
    COLOR_INTENSITY,
    COLOR_NONE,
};

typedef struct _RendererPCL RendererPCL;
struct _RendererPCL {
    BotRenderer renderer;
    
    lcm_t *lcm;
    BotParam *param;

    int have_data;
    
    BotPtrCircular   *data_circ;

    ripl_segment_list_t *segments; 
    ripl_normal_point_list_t *normals;

    ripl_xyz_point_list_t *xyz_points;
    ripl_xyzrgb_point_list_t *xyzrgb_points;
    ripl_cylinder_list_t *cylinders; 

    BotViewer         *viewer;
    BotGtkParamWidget *pw;   

    GMutex *mutex;
};

static void
on_cylinder_models (const lcm_recv_buf_t *rbuf, const char *channel,
             const ripl_cylinder_list_t *msg, void *user)
{
    RendererPCL *self = (RendererPCL *)user;
    g_assert(self);

    fprintf(stderr, ".");

    if(self->cylinders){
        ripl_cylinder_list_t_destroy(self->cylinders);
    }
    self->cylinders = ripl_cylinder_list_t_copy(msg);
    self->have_data = 1;

    bot_viewer_request_redraw (self->viewer);
}


static void
on_segment_points (const lcm_recv_buf_t *rbuf, const char *channel,
             const ripl_segment_list_t *msg, void *user)
{
    RendererPCL *self = (RendererPCL *)user;
    g_assert(self);

    fprintf(stderr, ".");

    if(self->segments){
        ripl_segment_list_t_destroy(self->segments);
    }
    self->segments = ripl_segment_list_t_copy(msg);
    self->have_data = 1;

    bot_viewer_request_redraw (self->viewer);
    
}

static void
on_xyz_points (const lcm_recv_buf_t *rbuf, const char *channel,
             const ripl_xyz_point_list_t *msg, void *user)
{
    RendererPCL *self = (RendererPCL *)user;
    g_assert(self);

    fprintf(stderr, ".");

    if(self->xyz_points){
        ripl_xyz_point_list_t_destroy(self->xyz_points);
    }
    self->xyz_points = ripl_xyz_point_list_t_copy(msg);
    self->have_data = 1;    
    bot_viewer_request_redraw (self->viewer);
}

static void
on_xyzrgb_points (const lcm_recv_buf_t *rbuf, const char *channel,
             const ripl_xyzrgb_point_list_t *msg, void *user)
{
    RendererPCL *self = (RendererPCL *)user;
    g_assert(self);

    fprintf(stderr, ".");

    if(self->xyzrgb_points){
        ripl_xyzrgb_point_list_t_destroy(self->xyzrgb_points);
    }
    self->xyzrgb_points = ripl_xyzrgb_point_list_t_copy(msg);
    self->have_data = 1;    
    bot_viewer_request_redraw (self->viewer);
}

static void
on_normal_points (const lcm_recv_buf_t *rbuf, const char *channel,
             const ripl_normal_point_list_t *msg, void *user)
{
    RendererPCL *self = (RendererPCL *)user;
    g_assert(self);

    fprintf(stderr, ".");

    if(self->normals){
        ripl_normal_point_list_t_destroy(self->normals);
    }
    self->normals = ripl_normal_point_list_t_copy(msg);
    self->have_data = 1;
    bot_viewer_request_redraw (self->viewer);
    
}

static void
renderer_pcl_destroy (BotRenderer *renderer)
{
    if (!renderer)
        return;

    RendererPCL *self = (RendererPCL *) renderer->user;
    if (!self)
        return;

    /*if (self->velodyne_data_circ)
        bot_ptr_circular_destroy (self->velodyne_data_circ);
    
    if (self->calib)
        velodyne_calib_free (self->calib);
    if (self->collector)
    velodyne_laser_return_collector_free (self->collector);*/
    
    free (self);
}

static void 
renderer_pcl_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererPCL *self = (RendererPCL*)renderer->user;
    g_assert(self);


    if(self->segments && bot_gtk_param_widget_get_bool (self->pw, DRAW_SEGMENTS)){
        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glPointSize(2.0);
        glBegin(GL_POINTS);
        double alpha = 1.0;//bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_ALPHA);
    
        for(int i=0; i< self->segments->no_segments; i++){
            ripl_seg_point_list_t *seg_msg = &self->segments->segments[i];
            //for now pick one color (to-do pick from palet)
            
            //glColor4d(1, 0, 1, alpha);

            fprintf(stderr, "Segment : %d => Points : %d \n", i, (int)seg_msg->no_points);
            
            //if(seg_msg->segment_id != -1)
            //  continue;
            
            //float *color = bot_color_util_jet(i/(double) self->segments->no_segments);
            //glColor4d(color[0], color[1], color[2], alpha);

            glColor3fv(bot_color_util_jet(i/(double) self->segments->no_segments));
            //free(color);
            /*fprintf(stderr, "Segment : %d => Points : %d Color : %f, %f, %f\n", i, (int)seg_msg->no_points, 
                    color[0], color[1], color[2]);
            
                    free(color);*/

            for(int j=0; j< seg_msg->no_points; j++){
                glVertex3d (seg_msg->points[j].xyz[0], seg_msg->points[j].xyz[1], seg_msg->points[j].xyz[2]);
                //glVertex3d (1,1,1);
            }
        }    
    
        glEnd();
        glPopAttrib();
    }

    if(self->cylinders && bot_gtk_param_widget_get_bool (self->pw, DRAW_CYLINDERS)){
        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glPointSize(2.0);
        //glBegin(GL_POINTS);
        double alpha = 1.0;
    
        for(int i=0; i< self->cylinders->count; i++){
            ripl_cylinder_model_t *cyl = &self->cylinders->list[i];           

            fprintf(stderr, "Cylinder : %d  => \n", i);
            
            float *color = bot_color_util_jet(i/(double) self->cylinders->count);
            glColor4d(color[0], color[1], color[2], alpha);

            /*glPushMatrix();         
            glTranslatef(x0, y0, 0);
            bot_gl_draw_circle(0.5);
            glPopMatrix();*/
            glBegin(GL_LINES);
            glVertex3d (cyl->line_point[0], cyl->line_point[1], cyl->line_point[2]);
            glVertex3d (cyl->line_point[0] + cyl->line_direction[0], cyl->line_point[1] + cyl->line_direction[1], cyl->line_point[2] + cyl->line_direction[2]);
            glEnd();
        }
        
        glPopAttrib();
    }

    if(self->segments && bot_gtk_param_widget_get_bool (self->pw, DRAW_SEGMENT_PLANES)){
        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glPointSize(2.0);
        glBegin(GL_LINES);
        double alpha = 1.0;//bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_ALPHA);
    
        for(int i=0; i< self->segments->no_segments; i++){
            ripl_seg_point_list_t *seg_msg = &self->segments->segments[i];
            fprintf(stderr, "Plane Segment : %d => Points : %d \n", i, (int)seg_msg->no_points);

            float *color = bot_color_util_jet(i/(double) self->segments->no_segments);
            glColor4d(color[0], color[1], color[2], alpha);
            //free(color);

            
            glVertex3d (seg_msg->centroid[0], seg_msg->centroid[1], seg_msg->centroid[2]);

            

            double a = seg_msg->coefficients[0]; 
            double b = seg_msg->coefficients[1]; 
            double c = seg_msg->coefficients[2]; 
            double d = seg_msg->coefficients[3]; 

            double r = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

            glVertex3d (seg_msg->centroid[0] + a / r, seg_msg->centroid[1] + b / r , seg_msg->centroid[2] + c /r );

            //double k = a * seg_msg->centroid[0] + b * seg_msg->centroid[1] + c *seg_msg->centroid[2]; 

            
            //glVertex3d (seg_msg->points[j].xyz[0], seg_msg->points[j].xyz[1], seg_msg->points[j].xyz[2]);
        }
        glEnd();
        glPopAttrib();
    }

    if(self->normals && bot_gtk_param_widget_get_bool (self->pw, DRAW_NORMALS)){      
        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);

        double alpha = 1.0;
        double scale = 0.05;
        
        glPointSize(1.0f);

        glColor4d(1.0, 0.0, 0, alpha);

        for(int i=0; i< self->normals->no_points; i++){
            glBegin(GL_LINE_STRIP);
            glVertex3d (self->normals->points[i].xyz[0], 
                        self->normals->points[i].xyz[1],
                        self->normals->points[i].xyz[2]);

            glVertex3d (self->normals->points[i].xyz[0] + self->normals->points[i].normals[0] * scale,
                        self->normals->points[i].xyz[1] + self->normals->points[i].normals[1] * scale,
                        self->normals->points[i].xyz[2] + self->normals->points[i].normals[2]*scale);
            glEnd();            
        }

        if(bot_gtk_param_widget_get_bool (self->pw, DRAW_XYZ)){
            glPointSize(2.0f);

            glBegin(GL_POINTS);
            glColor4d(0.0, .0, 1.0, alpha);
            
            for(int i=0; i< self->normals->no_points; i++){
                glVertex3d (self->normals->points[i].xyz[0], 
                            self->normals->points[i].xyz[1],
                            self->normals->points[i].xyz[2]);
            }
            glEnd();         
        }
        glPopAttrib();
    }

    if(self->xyz_points && bot_gtk_param_widget_get_bool (self->pw, DRAW_XYZ)){      
        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);

        double alpha = 1.0;
        double scale = 0.05;
        
        glPointSize(2.0f);

        glBegin(GL_POINTS);
        glColor4d(0.0, .0, 1.0, alpha);
                
        for(int i=0; i< self->xyz_points->no_points; i++){
            /*fprintf(stderr,"%f,%f,%f\n", self->xyz_points->points[i].xyz[0], 
                        self->xyz_points->points[i].xyz[1],
                        self->xyz_points->points[i].xyz[2]);*/
            /*if(self->xyz_points->points[i].xyz[0] > 2 || self->xyz_points->points[i].xyz[1] > 2 || self->xyz_points->points[i].xyz[0] > 2){
                fprintf(stderr,"Large values\n");
                }*/
            glVertex3d (self->xyz_points->points[i].xyz[0], 
                        self->xyz_points->points[i].xyz[1],
                        self->xyz_points->points[i].xyz[2]);
        }
        glEnd();
        glPopAttrib();
    }
    if(self->xyzrgb_points && bot_gtk_param_widget_get_bool (self->pw, DRAW_XYZRGB)){      
        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);

        double alpha = 1.0;
        double scale = 0.05;
        
        glPointSize(2.0f);

        glBegin(GL_POINTS);
                        
        for(int i=0; i< self->xyzrgb_points->no_points; i++){
            /*uint8_t r1 = 0, b1 = 0, g1 = 255;
            
            uint32_t rgb =  (uint32_t)r1  << 16 |
            (uint32_t)g1 << 8 | (uint32_t)b1;//*/
            uint32_t rgb = (uint32_t)self->xyzrgb_points->points[i].rgb; 

            uint8_t r = rgb >> 16;
            uint8_t g = rgb >> 8;
            uint8_t b = rgb;

            
            

            glColor4d(r/255.0, g/255.0, b/255.0, alpha);
            glVertex3d (self->xyzrgb_points->points[i].xyz[0], 
                        self->xyzrgb_points->points[i].xyz[1],
                        self->xyzrgb_points->points[i].xyz[2]);
        }
        glEnd();
        glPopAttrib();
    }

}


static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererPCL *self = user;
    bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPCL *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPCL *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererPCL *
renderer_pcl_new (BotViewer *viewer, BotParam * param)
{    
    RendererPCL *self = (RendererPCL*) calloc (1, sizeof (*self));

    self->viewer = viewer;

    BotRenderer *renderer = &self->renderer;
    renderer->draw = renderer_pcl_draw;
    renderer->destroy = renderer_pcl_destroy;
    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        fprintf (stderr,"Error: setup_renderer_laser() failed to get global lcm object\n");
        renderer_pcl_destroy (renderer);
        return NULL;
    }

    self->param = param;
    if (!self->param) {
        fprintf (stderr,"Error: setup_renderer_laser() failed to get BotParam instance\n");
        renderer_pcl_destroy (renderer);
        return NULL;
    }

    

    self->mutex = g_mutex_new ();
    
    //    self->data_circ = bot_ptr_circular_new (DATA_CIRC_SIZE,
    //                                               circ_free_velodyne_data, self);
    
    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);
    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
            G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
            G_CALLBACK (on_save_preferences), self);

    bot_gtk_param_widget_add_booleans (self->pw, 
                                       0,
                                       DRAW_NORMALS, 0, NULL);
    
    bot_gtk_param_widget_add_booleans (self->pw, 
                                       0,
                                       DRAW_SEGMENTS, 0, NULL);

    bot_gtk_param_widget_add_booleans (self->pw, 
                                       0,
                                       DRAW_CYLINDERS, 0, NULL);

    bot_gtk_param_widget_add_booleans (self->pw, 
                                       0,
                                       DRAW_SEGMENT_PLANES, 0, NULL);

    bot_gtk_param_widget_add_booleans (self->pw, 
                                       0,
                                       DRAW_XYZ, 0, NULL);

    bot_gtk_param_widget_add_booleans (self->pw, 
                                       0,
                                       DRAW_XYZRGB, 0, NULL);
    
    
    //velodyne_t_subscribe (self->lcm, lcm_channel, on_velodyne, self);
    self->segments = NULL;
    self->normals = NULL;
    self->xyz_points = NULL;
    self->xyzrgb_points = NULL;
    ripl_segment_list_t_subscribe(self->lcm, "PCL_SEGMENT_LIST", on_segment_points, self);

    ripl_normal_point_list_t_subscribe(self->lcm, "PCL_NORMAL_LIST", on_normal_points, self);

    ripl_xyz_point_list_t_subscribe(self->lcm, "PCL_XYZ_LIST", on_xyz_points, self);
    ripl_xyzrgb_point_list_t_subscribe(self->lcm, "PCL_XYZRGB_LIST", on_xyzrgb_points, self);

    ripl_cylinder_list_t_subscribe(self->lcm, "CYLINDER_LIST", on_cylinder_models, self);

    return self;
}

void
setup_renderer_pcl (BotViewer *viewer, int priority, BotParam * param)
{
    RendererPCL *self = renderer_pcl_new (viewer, param);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
