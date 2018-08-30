/*
 * Renders a set of scrolling plots in the top right corner of the window
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gdk/gdkkeysyms.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <bot_param/param_client.h>

#include <lcmtypes/ripl_velocity_msg_t.h>
#include <lcmtypes/ripl_robot_status_t.h>
#include <lcmtypes/ripl_raw_odometry_msg_t.h>

#define PARAM_NAME_GRAPH_TIMESPAN "Time span"
#define PARAM_NAME_FREEZE "Freeze"
#define PARAM_NAME_SIZE "Size"
#define PARAM_DRAW_STATUS "Draw Status"
#define PARAM_NAME_RENDER_TRANS_VEL "Trans Velocity"
#define PARAM_NAME_RENDER_ROT_VEL "Rot Velocity"
#define PARAM_NAME_SHOW_LEGEND "Show Legends"

#define REDRAW_THRESHOLD_UTIME 5000000
// The following are used to define the ploy y-axis limits unless
// otherwise specified in the param file
#define MAX_RV 0.7
#define MAX_TV 0.5

typedef struct _RendererRobotStatus RendererRobotStatus;

struct _RendererRobotStatus {
    lcm_t *lcm;

    BotRenderer renderer;
    BotViewer *viewer;

    BotGtkParamWidget    *pw;

    BotGlScrollPlot2d *trans_vel_plot;
    BotGlScrollPlot2d *rot_vel_plot;

    ripl_robot_status_t *robot_status;

    int64_t  vel_cmd_utime_last;
    uint64_t max_utime;
    double default_tv;
    double max_rv;
};

static void
update_xaxis (RendererRobotStatus *self, uint64_t utime)
{
    if ((utime < self->max_utime) &&
        (utime > self->max_utime - REDRAW_THRESHOLD_UTIME)) return;

    self->max_utime = utime;
    double timestamp = self->max_utime * 1e-6;
    bot_gl_scrollplot2d_add_point (self->rot_vel_plot, "0", timestamp, 0.0);
}




static void
on_robot_status (const lcm_recv_buf_t * buf, const char *channel,
                 const ripl_robot_status_t *msg, void *user_data)
{
    RendererRobotStatus *self = (RendererRobotStatus*) user_data;
    if (self->robot_status)
        ripl_robot_status_t_destroy (self->robot_status);
    self->robot_status = ripl_robot_status_t_copy (msg);
}


static void
on_velocity_msg (const lcm_recv_buf_t *rbuf, const char *channel,
                 const ripl_velocity_msg_t *msg, void *user)
{
    RendererRobotStatus *self = (RendererRobotStatus *) user;

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE))
        return;

    update_xaxis(self, msg->utime);

    self->vel_cmd_utime_last = msg->utime;
    double timestamp = msg->utime * 1e-6;

    bot_gl_scrollplot2d_add_point (self->trans_vel_plot, "requested", timestamp, msg->tv);
    bot_gl_scrollplot2d_add_point (self->rot_vel_plot, "requested", timestamp, msg->rv * 180/M_PI);

    bot_viewer_request_redraw (self->viewer);
}

static void
on_raw_odometry_msg (const lcm_recv_buf_t *rbuf, const char *channel,
                 const ripl_raw_odometry_msg_t *msg, void *user)
{
    RendererRobotStatus *self = (RendererRobotStatus *) user;

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE))
        return;

    if (!self->vel_cmd_utime_last)
        return;

    // The Java process that publishes odometry messages has, in the past (i.e., old logs)
    // published erroneous timestamps in the message
    int64_t utime;
    if (fabs(msg->utime - self->vel_cmd_utime_last) > 1e6)
        utime = self->vel_cmd_utime_last;
    else
        utime = msg->utime;

    update_xaxis(self, utime);

    double timestamp = utime * 1e-6;

    bot_gl_scrollplot2d_add_point (self->trans_vel_plot, "actual", timestamp, msg->tv);
    bot_gl_scrollplot2d_add_point (self->rot_vel_plot, "actual", timestamp, msg->rv*180/M_PI);

    bot_viewer_request_redraw (self->viewer);
}



static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name,
        RendererRobotStatus *self)
{
    if (! strcmp (name, PARAM_NAME_SHOW_LEGEND)) {
        BotGlScrollPlot2dLegendLocation legloc = BOT_GL_SCROLLPLOT2D_HIDDEN;
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_LEGEND)) {
            legloc = BOT_GL_SCROLLPLOT2D_TOP_RIGHT;
        }
        bot_gl_scrollplot2d_set_show_legend (self->trans_vel_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->rot_vel_plot, legloc);
    }

    bot_viewer_request_redraw (self->viewer);
}



static void
robot_status_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererRobotStatus *self = (RendererRobotStatus*) renderer->user;




    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];

    glGetDoublev (GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev (GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

    // Render the current robot status
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, viewport[2], 0, viewport[3]);

    glColor3f(1,1,1);
    int8_t state = self->robot_status ? self->robot_status->state : RIPL_ROBOT_STATUS_T_STATE_UNDEFINED;
    char *robot_string;
    switch (state)  {
    case RIPL_ROBOT_STATUS_T_STATE_RUN:
        robot_string = "RUN";
        break;
    case RIPL_ROBOT_STATUS_T_STATE_STOP:
        robot_string = "PAUSE";
        break;
    case RIPL_ROBOT_STATUS_T_STATE_MANUAL:
        robot_string = "MANUAL";
        break;
    case RIPL_ROBOT_STATUS_T_STATE_STANDBY:
        robot_string = "STANDBY";
        break;
    case RIPL_ROBOT_STATUS_T_STATE_ERROR:
        robot_string = "ERROR";
        break;
    default:
    case RIPL_ROBOT_STATUS_T_STATE_UNDEFINED:
        robot_string = "UNDEFINED";
        break;
    }

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    double state_xyz[] = {50, 90, 100};

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_DRAW_STATUS)) {
        bot_gl_draw_text(state_xyz, NULL, robot_string,
                         //                     GLUTIL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES |
                         BOT_GL_DRAW_TEXT_JUSTIFY_CENTER |
                         BOT_GL_DRAW_TEXT_ANCHOR_VCENTER |
                         BOT_GL_DRAW_TEXT_ANCHOR_HCENTER |
                         BOT_GL_DRAW_TEXT_DROP_SHADOW);
    }

    if ((self->robot_status)&&(self->robot_status->comment)&&
        strlen(self->robot_status->comment)) {
        state_xyz[0] += 40;
        bot_gl_draw_text(state_xyz, NULL, self->robot_status->comment,
                         //                     GLUTIL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES |
                         BOT_GL_DRAW_TEXT_JUSTIFY_LEFT |
                         BOT_GL_DRAW_TEXT_ANCHOR_VCENTER |
                         BOT_GL_DRAW_TEXT_ANCHOR_LEFT |
                         BOT_GL_DRAW_TEXT_DROP_SHADOW);
    }

    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    if (!self->max_utime) {
        return;
    }

    double ts_max = self->max_utime * 1e-6;
    double ts_min = ts_max - bot_gtk_param_widget_get_double (self->pw, PARAM_NAME_GRAPH_TIMESPAN);

    bot_gl_scrollplot2d_set_xlim (self->trans_vel_plot, ts_min, ts_max);
    bot_gl_scrollplot2d_set_xlim (self->rot_vel_plot, ts_min, ts_max);

    int plot_width = bot_gtk_param_widget_get_int (self->pw, PARAM_NAME_SIZE);
    int plot_height = plot_width / 3;

    int x = viewport[2] - plot_width;
    int y = viewport[1];


    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_TRANS_VEL)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->trans_vel_plot,
                x, y, plot_width, plot_height);
        y += plot_height;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_ROT_VEL)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->rot_vel_plot,
                x, y, plot_width, plot_height);
        y += plot_height;
    }




}

static void
robot_status_free (BotRenderer *renderer)
{
    RendererRobotStatus *self = (RendererRobotStatus*) renderer;
    if (self->robot_status)
        ripl_robot_status_t_destroy (self->robot_status);

    if (self)
        free (self);

}

BotRenderer *renderer_robot_status_new (BotViewer *viewer, BotParam *param)
{
    RendererRobotStatus *self =
        (RendererRobotStatus*) calloc (1, sizeof (RendererRobotStatus));
    self->viewer = viewer;
    self->renderer.draw = robot_status_draw;
    self->renderer.destroy = robot_status_free;
    self->renderer.name = "Robot Status";
    self->renderer.user = self;
    self->renderer.enabled = 1;

    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);

    self->lcm = bot_lcm_get_global (NULL);

    int ret = -1;
    ret = bot_param_get_double (param, "motion_planner.speed_design.default_tv", &(self->default_tv));
    if (ret != 0)
        self->default_tv = MAX_TV;
    else
        self->default_tv = self->default_tv * 1.1; // Add a scale to BotParam value for plotting

    ret = bot_param_get_double (param, "motion_planner.speed_design.max_rv", &(self->max_rv));
    if (ret != 0)
        self->max_rv = MAX_RV;
    else
        self->max_rv = self->max_rv * 1.25; // Add a scale to BotParam value for plotting


    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    gtk_container_add (GTK_CONTAINER (self->renderer.widget),
                       GTK_WIDGET(self->pw));
    gtk_widget_show (GTK_WIDGET (self->pw));

    bot_gtk_param_widget_add_int (self->pw, PARAM_NAME_SIZE,
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 50, 800, 10, 250);
    bot_gtk_param_widget_add_double (self->pw, PARAM_NAME_GRAPH_TIMESPAN,
                                     BOT_GTK_PARAM_WIDGET_SLIDER, 1, 20, 0.5, 5);
    bot_gtk_param_widget_add_booleans (self->pw,
                                       BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_NAME_FREEZE, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, 0,
                                       PARAM_NAME_RENDER_TRANS_VEL, 1,
                                       PARAM_NAME_RENDER_ROT_VEL, 1,
                                       NULL);
    bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_NAME_SHOW_LEGEND, 0, PARAM_DRAW_STATUS, 0, NULL);

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);


    // translational velocity plot
    self->trans_vel_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->trans_vel_plot, "TV (m/s)");
    bot_gl_scrollplot2d_set_text_color   (self->trans_vel_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_bgcolor      (self->trans_vel_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_border_color (self->trans_vel_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->trans_vel_plot, 0, self->default_tv);
    bot_gl_scrollplot2d_add_plot    (self->trans_vel_plot, "requested", 1000);
    bot_gl_scrollplot2d_set_color   (self->trans_vel_plot, "requested", 0.7, 0, 0.7, 1);

    bot_gl_scrollplot2d_add_plot    (self->trans_vel_plot, "actual", 1000);
    bot_gl_scrollplot2d_set_color   (self->trans_vel_plot, "actual", 0, 0, 1, 1);

    //bot_gl_scrollplot2d_add_plot    (self->trans_vel_plot, "2500", 1000);
    //bot_gl_scrollplot2d_set_color   (self->trans_vel_plot, "2500", 0.8, 0.8, 0.8, 0.5);


    // Rotational velocity
    self->rot_vel_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->rot_vel_plot, "RV (deg/s)");
    bot_gl_scrollplot2d_set_text_color   (self->rot_vel_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_bgcolor      (self->rot_vel_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_border_color (self->rot_vel_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->rot_vel_plot, -self->max_rv*180/M_PI, self->max_rv*180/M_PI);
    bot_gl_scrollplot2d_add_plot    (self->rot_vel_plot, "requested", 1000);
    bot_gl_scrollplot2d_set_color   (self->rot_vel_plot, "requested", 0.7, 0, 0.7, 1);

    bot_gl_scrollplot2d_add_plot    (self->rot_vel_plot, "actual", 1000);
    bot_gl_scrollplot2d_set_color   (self->rot_vel_plot, "actual", 0, 0, 1, 1);

    // legends?
    BotGlScrollPlot2dLegendLocation legloc = BOT_GL_SCROLLPLOT2D_HIDDEN;
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_LEGEND)) {
        legloc = BOT_GL_SCROLLPLOT2D_TOP_RIGHT;
    }
    bot_gl_scrollplot2d_set_show_legend (self->trans_vel_plot, legloc);
    bot_gl_scrollplot2d_set_show_legend (self->rot_vel_plot, legloc);

    self->robot_status = NULL;

    // subscribe to LCM messages
    ripl_raw_odometry_msg_t_subscribe (self->lcm, "ODOMETRY", on_raw_odometry_msg, self);
    //ripl_velocity_msg_t_subscribe (self->lcm, "ROBOT_VELOCITY_STATUS", on_velocity_msg, self);
    ripl_velocity_msg_t_subscribe (self->lcm, "ROBOT_VELOCITY_CMD", on_velocity_msg, self);
    ripl_robot_status_t_subscribe (self->lcm, "ROBOT_STATUS", on_robot_status, self);

    return &self->renderer;
}

void setup_renderer_robot_status (BotViewer *viewer, BotParam *param, int priority)
{
    bot_viewer_add_renderer(viewer, renderer_robot_status_new(viewer, param), priority);
}
