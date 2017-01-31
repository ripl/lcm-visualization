#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <check_gridmap/check_gridmap.h>
#include <geom_utils/geometry.h>
#include "er_renderers.h"
#include "er_gl_utils.h"
#include <bot_core/bot_core.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <hr_lcmtypes/lcm_channel_names.h>

#define RENDERER_NAME "Verify Check Gridmap"

#define PARAM_VERIFY_POSE "Check Pose"
#define PARAM_RENDER_LCMGL "Render LCMGL"

#define DRAW_PERSIST_SEC 4

typedef struct _RendererVerifyCheckGridmap RendererVerifyCheckGridmap;

struct _RendererVerifyCheckGridmap {
    BotRenderer renderer;
    BotEventHandler ehandler;
    BotViewer *viewer;
    lcm_t *lcm;

    GMutex *mutex;

    BotGtkParamWidget *pw;

    check_gridmap_t *grid;

    gboolean check_gridmap_render_lcmgl;

    int dragging;
    int active;
    point2d_t drag_start_local;
    point2d_t drag_finish_local;

    point2d_t particle_mean;
    double theta;
    double particle_std;

    int64_t max_draw_utime;
};

static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) renderer;
    int64_t now = bot_timestamp_now();
    if(!self->dragging && now > self->max_draw_utime)
        return;

    glColor3f(0, 1, 0);
    glPushMatrix();
    glTranslatef(self->particle_mean.x, self->particle_mean.y, 0);
    bot_gl_draw_circle(self->particle_std);

    glBegin(GL_LINE_STRIP);  
    glVertex2f(0.0,0.0);

    glVertex2f(self->particle_std*cos(self->theta),self->particle_std*sin(self->theta));
    glEnd();

    glPopMatrix();
}

static void
recompute_particle_distribution(RendererVerifyCheckGridmap *self)
{
    self->particle_mean = self->drag_start_local;
    double dx = self->drag_finish_local.x - self->drag_start_local.x;
    double dy = self->drag_finish_local.y - self->drag_start_local.y;

    double theta = atan2(dy,dx);
    self->theta = theta;

    self->particle_std = sqrt(dx*dx + dy*dy);
    self->max_draw_utime = bot_timestamp_now() + DRAW_PERSIST_SEC * 1000000;
}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
             const double ray_dir[3], const GdkEventButton *event)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) ehandler->user;

    self->dragging = 0;

    if(ehandler->picking==0)
        return 0;

    if(self->active==0)
        return 0;

    if(event->button != 1)
        return 0;

    point2d_t click_pt_local;
  
    if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
                                           POINT3D(ray_dir), 0, &click_pt_local)) {
        bot_viewer_request_redraw(self->viewer);
        self->active = 0;
        return 0;
    }

    self->dragging = 1;

    self->drag_start_local = click_pt_local;
    self->drag_finish_local = click_pt_local;

    recompute_particle_distribution(self);

    bot_viewer_request_redraw(self->viewer);
    return 1;
}

static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventButton *event)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) ehandler->user;
    
    if (self->dragging) {
        self->dragging = 0;
    }
    if (self->active != 0) {
        // check drag points and publish
        
        double dpos = 0.1;
        double x1 = self->particle_mean.x - dpos/2*cos(self->theta);
        double x2 = self->particle_mean.x + dpos/2*cos(self->theta);
        double y1 = self->particle_mean.y - dpos/2*sin(self->theta);
        double y2 = self->particle_mean.y + dpos/2*sin(self->theta);
        
        int is_forward = 1;
        int failsafe = 0;
        struct check_path_result path_res;
        
        check_gridmap_check_path (self->grid, is_forward, failsafe,
                                  x1, y1, self->theta,
                                  x2, y2, self->theta,
                                  &path_res);
        
        fprintf(stdout, "cost : %.4f, max = %.4f, obs_max = %.4f\n", 
                path_res.cost, path_res.max, path_res.obs_max);
        

        bot_viewer_set_status_bar_message(self->viewer, "");
        self->active = 0;
        bot_gtk_param_widget_set_enabled (self->pw, PARAM_RENDER_LCMGL, TRUE);
        return 1;
    }
    else
        ehandler->picking = 0;
    
    return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventMotion *event)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) ehandler->user;
    
    if(!self->dragging || self->active==0)
        return 0;
    
    point2d_t drag_pt_local;
    if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
                                           POINT3D(ray_dir), 0, &drag_pt_local)) {
        return 0;
    }
    
    self->drag_finish_local = drag_pt_local;
    recompute_particle_distribution(self);
    
    bot_viewer_request_redraw(self->viewer);
    return 1;
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
                      const GdkEventKey *event)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) ehandler->user;
    
    if(event->keyval == GDK_Escape) {
        self->active = 0;
        ehandler->picking = 0;
        bot_gtk_param_widget_set_enabled (self->pw, PARAM_RENDER_LCMGL, TRUE);
        bot_viewer_set_status_bar_message(self->viewer, "");
    }
    
    return 0;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) user;
    if(!strcmp(name, PARAM_VERIFY_POSE)) {
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        check_gridmap_update (self->grid);
        self->active = 1;
        bot_gtk_param_widget_set_enabled (self->pw, PARAM_RENDER_LCMGL, FALSE);
    }

    if (self->check_gridmap_render_lcmgl != bot_gtk_param_widget_get_bool (self->pw, PARAM_RENDER_LCMGL)) {
        self->active = 0;
        self->check_gridmap_render_lcmgl = bot_gtk_param_widget_get_bool (self->pw, PARAM_RENDER_LCMGL);
        bot_gtk_param_widget_set_enabled (self->pw, PARAM_VERIFY_POSE, FALSE);
        fprintf (stdout, "Destroying the gridmap\n");
        check_gridmap_destroy (self->grid);
        fprintf (stdout, "Creating a new gridmap......\n");
        self->grid = check_gridmap_create (0, self->check_gridmap_render_lcmgl, FALSE, FALSE, FALSE, FALSE, FALSE, 0.3);
        fprintf (stdout, "..... done\n");
        bot_gtk_param_widget_set_enabled (self->pw, PARAM_VERIFY_POSE, TRUE);
    }

}

static void
_free (BotRenderer *renderer)
{
    free (renderer);
}

BotRenderer *renderer_verify_check_gridmap_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererVerifyCheckGridmap *self = (RendererVerifyCheckGridmap*) calloc (1, sizeof (RendererVerifyCheckGridmap));
    self->viewer = viewer;
    self->renderer.draw = _draw;
    self->renderer.destroy = _free;
    self->renderer.name = RENDERER_NAME;
    self->renderer.user = self;
    self->renderer.enabled = 1;

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->key_press = key_press;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    self->lcm = lcm; //globals_get_lcm_full(NULL,1);

    // Create the check gridmap
    self->check_gridmap_render_lcmgl = TRUE;
    fprintf (stdout, "Creating the gridmap.....\n");
    self->grid = check_gridmap_create (0, self->check_gridmap_render_lcmgl, FALSE, FALSE, FALSE, FALSE, FALSE, 0.1);
    fprintf (stdout, "..... done\n");

    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_VERIFY_POSE, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_RENDER_LCMGL, 
                                       self->check_gridmap_render_lcmgl, NULL);

    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->renderer.widget = GTK_WIDGET(self->pw);

    self->active = 0;

    return &self->renderer;
}

void verify_check_gridmap_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    bot_viewer_add_renderer(viewer, renderer_verify_check_gridmap_new(viewer, render_priority, lcm), 
                            render_priority);
}
