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
#include <math.h>

#include <bot_core/bot_core.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gtk_util.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include "lcmtypes/hr_lcmtypes.h" 

#define DTOR M_PI/180
#define RTOD 180/M_PI

#define MAX_REFERSH_RATE_USEC 30000 // about 1/30 of a second

#define RENDERER_NAME "Topological Graph"
#define DATA_CIRC_SIZE 10
#define POSE_DATA_CIRC_SIZE 200 

#define PARAM_COLOR_MENU "Color"
#define PARAM_HISTORY_LENGTH "Scan Hist. Len."
#define PARAM_HISTORY_FREQUENCY "Scan Hist. Freq."
#define PARAM_POINT_SIZE "Point Size" 
#define PARAM_POINT_ALPHA "Point Alpha" 

#define DRAW_XYZ "xyz"





enum {
    COLOR_Z,
    COLOR_INTENSITY,
    COLOR_NONE,
};

typedef struct _RendererTopoGraph RendererTopoGraph;
struct _RendererTopoGraph {
    BotRenderer renderer;
    
    lcm_t *lcm;
    BotParam *param;

    int have_data;
    
    BotPtrCircular   *data_circ;

    erlcm_xyz_point_list_t *xyz_points;

    BotViewer         *viewer;
    BotGtkParamWidget *pw;   

    GMutex *mutex;
};

static void
on_xyz_points (const lcm_recv_buf_t *rbuf, const char *channel,
             const erlcm_xyz_point_list_t *msg, void *user)
{
    //fprintf(stderr, "GOT MESSAGE");
    RendererTopoGraph *self = (RendererTopoGraph *)user;
    g_assert(self);

    fprintf(stderr, ".");

    if(self->xyz_points){
        erlcm_xyz_point_list_t_destroy(self->xyz_points);
    }
    self->xyz_points = erlcm_xyz_point_list_t_copy(msg);
    self->have_data = 1;    
    bot_viewer_request_redraw (self->viewer);
}



static void
renderer_topological_graph_destroy (BotRenderer *renderer)
{
    if (!renderer)
        return;

    RendererTopoGraph *self = (RendererTopoGraph *) renderer->user;
    if (!self)
        return;
    
    free (self);
}

static void 
renderer_topological_graph_draw (BotViewer *viewer, BotRenderer *renderer)
{
    //fprintf(stderr, "DRAWING");
    
    RendererTopoGraph *self = (RendererTopoGraph*)renderer->user;
    g_assert(self);
    
    if(self->xyz_points && bot_gtk_param_widget_get_bool (self->pw, DRAW_XYZ)){   
        //fprintf(stderr, "ACTUALLY DRAWING");   
        
        for(int i=0; i< self->xyz_points->no_points; i++){
            glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
            glEnable (GL_DEPTH_TEST);
            glEnable(GL_BLEND);

            double alpha = 1.0;
            double scale = 0.05;
        
            glPointSize(4.0f);
        
            double radius = .2;
        
            glBegin(GL_TRIANGLE_FAN);
            glColor4d(0.0, .0, 1.0, alpha);
            double x1 = self -> xyz_points -> points[i].xyz[0];
            double y1 = self -> xyz_points -> points[i].xyz[1];
            double z1 = self -> xyz_points -> points[i].xyz[2];
            glVertex3d(x1,y1,z1);
            for(double angle = 0; angle <=2*M_PI; angle += M_PI/20){
               glVertex3d(x1 + sin(angle) * radius, y1 + cos(angle) * radius, z1);
               
            }
            
            glEnd();
            glPopAttrib();
            
            if (i < self -> xyz_points -> no_points -1){
                  glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
                  glEnable (GL_DEPTH_TEST);
                  glEnable(GL_BLEND);

                  double alpha = 1.0;
                  double scale = 0.05;
        
                  glPointSize(4.0f);
                  glBegin(GL_LINES);
                  glColor4d(0.0, .0, 1.0, alpha);
                  glVertex3d( self->xyz_points->points[i].xyz[0],
                              self->xyz_points->points[i].xyz[1],
                              self->xyz_points->points[i].xyz[2]);
                  glVertex3d( self->xyz_points->points[i+1].xyz[0],
                              self->xyz_points->points[i+1].xyz[1],
                              self->xyz_points->points[i+1].xyz[2]);
                  glEnd();
                  glPopAttrib();
            }
        }
    }
}


static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererTopoGraph *self = user;
    bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererTopoGraph *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererTopoGraph *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static RendererTopoGraph *
renderer_topological_graph_new (BotViewer *viewer, BotParam * param)
{    
    RendererTopoGraph *self = (RendererTopoGraph*) calloc (1, sizeof (*self));

    self->viewer = viewer;

    BotRenderer *renderer = &self->renderer;
    renderer->draw = renderer_topological_graph_draw;
    renderer->destroy = renderer_topological_graph_destroy;
    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        fprintf (stderr,"Error: setup_renderer_laser() failed to get global lcm object\n");
        renderer_topological_graph_destroy (renderer);
        return NULL;
    }

    self->param = param;
    if (!self->param) {
        fprintf (stderr,"Error: setup_renderer_laser() failed to get BotParam instance\n");
        renderer_topological_graph_destroy (renderer);
        return NULL;
    }

    

    self->mutex = g_mutex_new ();
     
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
                                       DRAW_XYZ, 0, NULL);

    self->xyz_points = NULL;

    erlcm_xyz_point_list_t_subscribe(self->lcm, "TOPOLOGICAL_GRAPH_XYZ_LIST", on_xyz_points, self);
   
    return self;
}

void
setup_renderer_topological_graph (BotViewer *viewer, int priority, BotParam * param)
{
    RendererTopoGraph *self = renderer_topological_graph_new (viewer, param);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
