#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <hr_lcmtypes/lcm_channel_names.h>

#include "gl_utils.h"

#define RENDERER_NAME "ER Waypoints"

#define PARAM_WAYPOINT_CURRENT "Show Curr. Waypt"
#define PARAM_WAYPOINT_GOAL "Show Goal. Waypt"
#define PARAM_CLICK_WAYPOINT_TYPE "Click Waypt Type"
#define PARAM_CLICK_WAYPOINT_HEIGHT "Height"
#define PARAM_CLICK_VELOCITY "Velocity"
#define PARAM_CLICK_WAYPOINT_YAW "Yaw"
#define PARAM_HEIGHT_SWEEP_HEIGHT "Heigh Sweep Height"

#define PARAM_CLICK_VELOCITY_MIN 0.1
#define PARAM_CLICK_VELOCITY_MAX 3.0
#define PARAM_CLICK_VELOCITY_INC .01

#define PARAM_CLICK_WAYPOINT_HEIGHT_MIN 0
#define PARAM_CLICK_WAYPOINT_HEIGHT_MAX 6
#define PARAM_CLICK_WAYPOINT_HEIGHT_INC 0.1

#define PARAM_HEIGHT_SWEEP_HEIGHT_MIN 1
#define PARAM_HEIGHT_SWEEP_HEIGHT_MAX 6
#define PARAM_HEIGHT_SWEEP_HEIGHT_INC 0.1

//rate limit keyboard commands to this much
#define MAX_KEYBOARD_XY_VEL 0.5
#define MAX_KEYBOARD_Z_VEL  0.2
#define MAX_KEYBOARD_YAW_VEL 0.25

typedef enum _click_waypt_t {
    PLANNER_WAYPT, CONTROLLER_WAYPT
} click_waypt_t;

typedef struct _RendererWaypt {
    BotRenderer renderer;
    BotEventHandler ehandler;

    lcm_t *lc;

    BotViewer *viewer;
    BotGtkParamWidget *pw;

    bot_core_pose_t *last_curr_waypt;
    bot_core_pose_t *last_goal_waypt;
    ripl_waypoint_msg_t_subscription_t *waypt_subscription;
    int max_draw_poses;
    int send_waypts;
    int show_waypt;
    gboolean param_draw_current_waypt;
    gboolean param_draw_goal_waypt;

    click_waypt_t param_click_waypt_type;
    double last_xyzt[4]; // last mouse press
    double hover_xyz[4];
    double mouse_worldpos[3];
    double velocity;
    double height_sweep_height;

    int64_t last_body_relative_waypt_time;
    int64_t last_yaw_relative_waypt_time;
    int64_t last_height_relative_waypt_time;

} RendererWaypt;

static void on_waypoint(const lcm_recv_buf_t *rbuf, const char *channel, const ripl_waypoint_msg_t *msg, void *user)
{
    RendererWaypt *self = (RendererWaypt*) user;

    double rpy[3] = { 0, 0, msg->pos.roll};

    double posquat[4];
    bot_roll_pitch_yaw_to_quat(rpy, posquat);

    bot_core_pose_t temp_waypt;
    /*    if (!self->last_waypt){
          self->last_waypt = (bot_core_pose_t*) calloc(1, sizeof(bot_core_pose_t));
          }*/

    temp_waypt.pos[0] = msg->pos.x;
    temp_waypt.pos[1] = msg->pos.y;
    temp_waypt.pos[2] = msg->pos.z;
    temp_waypt.orientation[0] = posquat[0];
    temp_waypt.orientation[1] = posquat[1];
    temp_waypt.orientation[2] = posquat[2];
    temp_waypt.orientation[3] = posquat[3];

    if (strcmp(channel, WAYPOINT_CURR_CHANNEL) == 0) {
        if (self->last_curr_waypt != NULL)
            bot_core_pose_t_destroy(self->last_curr_waypt);
        self->last_curr_waypt = bot_core_pose_t_copy(&temp_waypt);
    }

    if (strcmp(channel, WAYPOINT_GOAL_CHANNEL) == 0) {
        if (self->last_goal_waypt != NULL)
            bot_core_pose_t_destroy(self->last_goal_waypt);
        self->last_goal_waypt = bot_core_pose_t_copy(&temp_waypt);
    }

    bot_viewer_request_redraw(self->viewer);
}

static void quad_free(BotRenderer *super)
{
    RendererWaypt *self = (RendererWaypt*) super->user;
    free(self);
}

static void quad_draw(BotViewer *viewer, BotRenderer *super)
{
    RendererWaypt *self = (RendererWaypt*) super->user;

    if (self->param_draw_current_waypt && self->last_curr_waypt != NULL) {
        float blue[3] = { 0, 0, 1 };
        float green[3] = { 0, 1, 0 };
        draw_boxy_quad(self->last_curr_waypt->pos, self->last_curr_waypt->orientation, blue, green);
    }

    if (self->param_draw_goal_waypt && self->last_goal_waypt != NULL) {
        float blue[3] = { 0, 0, 1 };
        float yellow[3] = { 1, 1, 0 };
        draw_boxy_quad(self->last_goal_waypt->pos, self->last_goal_waypt->orientation, blue, yellow);
    }

    if (self->show_waypt) {
        char buf[256];
        glColor3f(1, 1, 1);
        if (self->param_click_waypt_type == CONTROLLER_WAYPT)
            sprintf(buf, "CONT: %.3f %.3f %.1f %.0f", self->hover_xyz[0], self->hover_xyz[1], self->last_xyzt[2],
                    self->last_xyzt[3]);
        else if (self->param_click_waypt_type == PLANNER_WAYPT)
            sprintf(buf, "NAV: %.3f %.3f %.1f %.0f", self->hover_xyz[0], self->hover_xyz[1], self->last_xyzt[2],
                    self->last_xyzt[3]);
        bot_gl_draw_text(self->mouse_worldpos, GLUT_BITMAP_HELVETICA_12, buf, BOT_GL_DRAW_TEXT_DROP_SHADOW);
    }

}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererWaypt *self = (RendererWaypt*) user;

    self->param_draw_current_waypt = bot_gtk_param_widget_get_bool(pw, PARAM_WAYPOINT_CURRENT);
    self->param_draw_goal_waypt = bot_gtk_param_widget_get_bool(pw, PARAM_WAYPOINT_GOAL);
    self->param_click_waypt_type = bot_gtk_param_widget_get_enum(pw, PARAM_CLICK_WAYPOINT_TYPE);
    self->last_xyzt[2] = bot_gtk_param_widget_get_double(pw, PARAM_CLICK_WAYPOINT_HEIGHT);
    self->last_xyzt[3] = bot_to_radians(bot_gtk_param_widget_get_double(pw, PARAM_CLICK_WAYPOINT_YAW));
    self->velocity = bot_gtk_param_widget_get_double(pw, PARAM_CLICK_VELOCITY);
    self->height_sweep_height = bot_gtk_param_widget_get_double(pw, PARAM_HEIGHT_SWEEP_HEIGHT);
    bot_viewer_request_redraw(self->viewer);
}

static int mouse_press(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3],
                       const double ray_dir[3], const GdkEventButton *event)
{
    RendererWaypt *self = (RendererWaypt*) ehandler->user;

    double dist_to_ground = -ray_start[2] / ray_dir[2];

    self->last_xyzt[0] = ray_start[0] + dist_to_ground * ray_dir[0];
    self->last_xyzt[1] = ray_start[1] + dist_to_ground * ray_dir[1];

    if ((event->state & GDK_CONTROL_MASK) && event->button == 1) {
        //LEFT BUTTON: Go to waypoint
        ripl_waypoint_msg_t waypt_msg;
        //ripl_controller_set_noop(&waypt_msg);
        waypt_msg.waypt_type = RIPL_WAYPOINT_MSG_T_TYPE_WAYPT;
        waypt_msg.pos.x = self->last_xyzt[0];
        waypt_msg.pos.y = self->last_xyzt[1];

        waypt_msg.vel.x = self->velocity;
        waypt_msg.vel.y = self->velocity;

        waypt_msg.nonce = random();
        waypt_msg.utime = bot_timestamp_now();
        waypt_msg.sender = RIPL_WAYPOINT_MSG_T_SENDER_VIEWER;//QUAD_WAYPOINT_T_SENDER_VIEWER;

        /*if (self->param_click_waypt_type == CONTROLLER_WAYPT) {
          ripl_controller_sanity_check_and_publish(self->lc, WAYPOINT_COMMAND_CHANNEL, &waypt_msg);
          fprintf(stderr, "Controller click %.3f %.3f %.3f\n", self->last_xyzt[0], self->last_xyzt[1], self->last_xyzt[2]);
          }
          else if (self->param_click_waypt_type == PLANNER_WAYPT) {
          waypt_msg.xyzt[2] = self->last_xyzt[2];
          waypt_msg.xyzt[3] = self->last_xyzt[3];
          ripl_controller_sanity_check_and_publish(self->lc, WAYPOINT_PLANNER_GOAL_CHANNEL, &waypt_msg);
          fprintf(stderr, "Planner waypoint click %.3f %.3f %.3f\n", self->last_xyzt[0], self->last_xyzt[1],
          self->last_xyzt[2]);
          }*/
    }
    else if ((event->state & GDK_CONTROL_MASK) && event->button == 3) {
        //RIGHT BUTTON: Face toward waypoint
        //compute the yaw...
        double xy[2] = { 0, 0 };
        if (self->last_curr_waypt != NULL) {
            xy[0] = self->last_curr_waypt->pos[0];
            xy[1] = self->last_curr_waypt->pos[1];
        }
        self->last_xyzt[3] = atan2(self->last_xyzt[1] - xy[1], self->last_xyzt[0] - xy[0]);

        /*quad_waypoint_t waypt_msg;
          ripl_controller_set_noop(&waypt_msg);
          waypt_msg.waypt_type = QUAD_WAYPOINT_T_TYPE_WAYPT;
          waypt_msg.xyzt[3] = self->last_xyzt[3];
          waypt_msg.nonce = random();
          waypt_msg.utime = bot_timestamp_now();
          waypt_msg.sender = QUAD_WAYPOINT_T_SENDER_VIEWER;
          ripl_controller_sanity_check_and_publish(self->lc, WAYPOINT_COMMAND_CHANNEL, &waypt_msg);
          fprintf(stderr, "Controller click %.3f %.3f %.3f\n", self->last_xyzt[0], self->last_xyzt[1], self->last_xyzt[2]);
        */
    }

    return 0;
}

static double pick_query(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3],
                         const double ray_dir[3])
{
    RendererWaypt *self = (RendererWaypt*) ehandler->user;

    memcpy(self->mouse_worldpos, ray_start, 3 * sizeof(double));
    double dist_to_ground = -ray_start[2] / ray_dir[2];

    self->hover_xyz[0] = ray_start[0] + dist_to_ground * ray_dir[0];
    self->hover_xyz[1] = ray_start[1] + dist_to_ground * ray_dir[1];
    self->ehandler.hovering = 1;
    return -1;
}

static void rateLimitHeightRelativeWaypts(RendererWaypt *self, double dh)
{
    int64_t now = bot_timestamp_now();

    /*double min_period = fabs(dh) / MAX_KEYBOARD_Z_VEL;
      if (now - self->last_height_relative_waypt_time > min_period * 1e6) {
      self->last_height_relative_waypt_time = now;
      ripl_controller_publish_relative_waypoint(self->lc, 0, 0, dh, 0, now);
      }*/

}
static void rateLimitYawRelativeWaypts(RendererWaypt *self, double dyaw)
{
    int64_t now = bot_timestamp_now();

    /*double min_period = fabs(dyaw) / MAX_KEYBOARD_YAW_VEL;
      if (now - self->last_yaw_relative_waypt_time > min_period * 1e6) {
      self->last_yaw_relative_waypt_time = now;
      ripl_controller_publish_relative_waypoint(self->lc, 0, 0, 0, dyaw, now);
      }*/

}

static void rateLimitBodyRelativeWaypts(RendererWaypt *self, double dx, double dy)
{
    int64_t now = bot_timestamp_now();
    /*double vec[2] = { dx, dy };
      double mag = bot_vector_magnitude_2d(vec);
      double min_period = mag / MAX_KEYBOARD_XY_VEL;
      if (now - self->last_body_relative_waypt_time > min_period * 1e6) {
      self->last_body_relative_waypt_time = now;
      ripl_controller_publish_body_relative_waypoint(self->lc, dx, dy, now);
      }*/
}

static void send_height_sweep_trajectory(RendererWaypt *self) {
    fprintf(stderr, "  - assembling height trajectory\n");
    /*
      quad_waypoint_list_t msg;
      msg.num_waypoints = 3;
      msg.waypoints = (quad_waypoint_t*) calloc(msg.num_waypoints, sizeof(quad_waypoint_t));

      if (self->last_curr_waypt == NULL) {
      fprintf(stderr, "Error: can't send height sweep until we get a pose\n");
      return;
      }

      // get robot's current theta
      double rpy[3];
      bot_quat_to_roll_pitch_yaw(self->last_curr_waypt->orientation, rpy);

      // make and copy 3 waypoints at the current position
      for(int i=0; i < msg.num_waypoints; i++) {
      msg.waypoints[i].utime = msg.utime;
      msg.waypoints[i].xyzt[0] = self->last_curr_waypt->pos[0];
      msg.waypoints[i].xyzt[1] = self->last_curr_waypt->pos[1];
      msg.waypoints[i].xyzt[2] = self->last_curr_waypt->pos[2];
      msg.waypoints[i].xyzt[3] = rpy[2];

      //copy over the velocity
      msg.waypoints[i].xyzt_dot[0] = self->velocity;
      msg.waypoints[i].xyzt_dot[1] = self->velocity;
      msg.waypoints[i].xyzt_dot[2] = self->velocity;
      msg.waypoints[i].xyzt_dot[3] = self->velocity;

      msg.waypoints[i].waypt_type = QUAD_WAYPOINT_T_TYPE_WAYPT;
      msg.waypoints[i].sender = QUAD_WAYPOINT_T_SENDER_VIEWER;
      msg.waypoints[i].utime = bot_timestamp_now();
      msg.waypoints[i].nonce = random();
      }

      // now adjust z values
      // sweep down
      msg.waypoints[0].xyzt[2] = 0.3;
      // sweep up
      msg.waypoints[1].xyzt[2] = self->height_sweep_height; //2.0;
      // back to center
      //   - leave as is

      fprintf(stderr, "  - publishing heights: %f\t%f\t%f\n", msg.waypoints[0].xyzt[2], msg.waypoints[1].xyzt[2], msg.waypoints[2].xyzt[2]);
      msg.utime = bot_timestamp_now();
      msg.nonce = random();
      quad_waypoint_list_t_publish(self->lc, PLANNED_WAYPOINTS_CHANNEL, &msg);

      // clear memory
      free(msg.waypoints);*/
}

static int key_press(BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event)
{

    double dxy = 0.1;
    double dz = 0.1;
    double dyaw = 0.05;

    RendererWaypt *self = (RendererWaypt*) ehandler->user;
    int keyval = event->keyval;
    switch (keyval) {
    case 'P':
    case 'p':
        self->show_waypt = !self->show_waypt;
        break;
    default:
        break;
    }
    return 0;
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererWaypt *self = user_data;
    bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererWaypt *self = user_data;
    bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

void setup_renderer_waypoint (BotViewer *viewer, int render_priority, lcm_t * lcm)
{
    RendererWaypt *self = (RendererWaypt*) calloc(1, sizeof(RendererWaypt));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = quad_draw;
    renderer->destroy = quad_free;

    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->key_press = key_press;
    ehandler->hover_query = pick_query;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    self->viewer = viewer;
    self->lc = lcm;
    self->last_curr_waypt = NULL;
    self->last_goal_waypt = NULL;
    self->param_click_waypt_type = CONTROLLER_WAYPT;

    self->last_xyzt[0] = 0;
    self->last_xyzt[1] = 0;
    self->last_xyzt[2] = 0;
    self->last_xyzt[3] = 0;

    self->waypt_subscription = ripl_waypoint_msg_t_subscribe(self->lc, WAYPOINT_CURR_CHANNEL, on_waypoint, self);
    self->waypt_subscription = ripl_waypoint_msg_t_subscribe(self->lc, WAYPOINT_GOAL_CHANNEL, on_waypoint, self);

    // TBD - WAYPOINT_COMMAND_CHANNEL

    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);

    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_WAYPOINT_CURRENT, 1, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_WAYPOINT_GOAL, 0, NULL);
    bot_gtk_param_widget_add_enum(self->pw, PARAM_CLICK_WAYPOINT_TYPE, BOT_GTK_PARAM_WIDGET_MENU,
                                  self->param_click_waypt_type, "Planner Waypt", PLANNER_WAYPT, "Controller Waypt", CONTROLLER_WAYPT, NULL);
    bot_gtk_param_widget_add_double(self->pw, PARAM_CLICK_WAYPOINT_HEIGHT, BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_CLICK_WAYPOINT_HEIGHT_MIN, PARAM_CLICK_WAYPOINT_HEIGHT_MAX, PARAM_CLICK_WAYPOINT_HEIGHT_INC, 1);

    bot_gtk_param_widget_add_double(self->pw, PARAM_CLICK_VELOCITY, BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_CLICK_VELOCITY_MIN, PARAM_CLICK_VELOCITY_MAX, PARAM_CLICK_VELOCITY_INC, .5);

    // height sweep max height
    bot_gtk_param_widget_add_double(self->pw, PARAM_HEIGHT_SWEEP_HEIGHT, BOT_GTK_PARAM_WIDGET_SLIDER,
                                    PARAM_HEIGHT_SWEEP_HEIGHT_MIN, PARAM_HEIGHT_SWEEP_HEIGHT_MAX, PARAM_HEIGHT_SWEEP_HEIGHT_INC, 2.0);

    bot_gtk_param_widget_add_int(self->pw, PARAM_CLICK_WAYPOINT_YAW, BOT_GTK_PARAM_WIDGET_SLIDER, -180, 180, 1, 0);

    self->param_draw_current_waypt = bot_gtk_param_widget_get_bool(self->pw, PARAM_WAYPOINT_CURRENT);
    self->param_draw_goal_waypt = bot_gtk_param_widget_get_bool(self->pw, PARAM_WAYPOINT_GOAL);

    gtk_widget_show_all(renderer->widget);

    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    on_param_widget_changed(self->pw, "", self);

    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
    g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
}
