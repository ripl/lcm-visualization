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
#include <geom_utils/geometry.h>

#include <bot_core/bot_core.h>
#include <bot_vis/gtk_util.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gl_util.h>
#include <bot_vis/scrollplot2d.h>

#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/bot2_param.h>

#define H_POSITION "H-Position"
#define V_POSITION "V-Position"

#define PARAM_START "Start"
#define PARAM_END "End"
#define PARAM_SEND_MANUAL "Send Manual Signal"
#define PARAM_SEND_POINT_REQUEST "Send Point Request"
#define PARAM_SEND_FRONT_FAULT "Send Front Drop"
#define PARAM_SEND_REAR_FAULT "Send Rear Drop"
#define PARAM_SEND_FOVIATION_COMMAND "Send Foviation Command"
#define RENDERER_NAME "Robot-Command"
#define PARAM_REGION_ID "Region ID"
#define PARAM_SEND_REGION "Send Region"

#define PARAM_SEGMENT_ID "Person Segment ID"
#define PARAM_SEND_SEGMENT "Send Segment"

#define PARAM_FLOOR_ID "Floor ID"
#define PARAM_SEND_FLOOR "Floor Region"

typedef struct _RendererRobotCommands RendererRobotCommands;

struct _RendererRobotCommands {
    lcm_t *lcm;

    BotRenderer renderer;
    BotViewer *viewer;
    BotEventHandler ehandler;

    BotGtkParamWidget    *pw;
    int64_t segment_utime;
    BotGlScrollPlot2d *trans_vel_plot;
    BotGlScrollPlot2d *rot_vel_plot;

    double pos[2];

    int64_t  vel_cmd_utime_last;
    uint64_t      max_utime;
};


static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, 
        RendererRobotCommands *self)
{
    if(!strcmp(name, H_POSITION)) {
        
        double raw_value = bot_gtk_param_widget_get_double(pw,H_POSITION);

        ripl_servo_position_list_t msg;

        msg.utime = bot_timestamp_now();
        msg.dof = 2; 
        msg.values = calloc(2, sizeof(double));
        msg.values[0] = raw_value; 
        msg.command_type = 0x0001;
        
        ripl_servo_position_list_t_publish (self->lcm, "HEAD_POSITION", &msg);
        free(msg.values);
    }
    if(!strcmp(name, V_POSITION)) {

        double raw_value = bot_gtk_param_widget_get_double(pw, V_POSITION);

        ripl_servo_position_list_t msg;

        msg.utime = bot_timestamp_now();
        msg.dof = 2; 
        msg.values = calloc(2, sizeof(double));
        msg.values[1] = raw_value; 
        msg.command_type = 0x0010;
        
        ripl_servo_position_list_t_publish (self->lcm, "HEAD_POSITION", &msg);
        free(msg.values);
    }

    if(!strcmp(name, PARAM_SEND_POINT_REQUEST)) {

        /*int start = bot_gtk_param_widget_get_int(pw, PARAM_START);
        int end = bot_gtk_param_widget_get_int(pw, PARAM_END);

        ripl_slam_node_select_t msg;
        msg.utime = bot_timestamp_now();
        msg.no_nodes = 2;
        msg.node_ind = (int32_t *) calloc(2,sizeof(int32_t));
        
        msg.node_ind[0] =  bot_gtk_param_widget_get_int(pw, PARAM_START);
        msg.node_ind[1] =  bot_gtk_param_widget_get_int(pw, PARAM_END);
        msg.type_of_nodes = RIPL_SLAM_NODE_SELECT_T_NODES_START_END;
        msg.mode = RIPL_SLAM_NODE_SELECT_T_PUBLISH_VOXEL_MAP;
        */
        int start = bot_gtk_param_widget_get_int(pw, PARAM_START);
        int end = bot_gtk_param_widget_get_int(pw, PARAM_END);

        ripl_slam_node_select_t msg;
        msg.utime = bot_timestamp_now();
        int count = end-start+1;
        msg.no_nodes = count;
        msg.node_ind = (int32_t *) calloc(count,sizeof(int32_t));
        
        msg.node_ind[0] =  bot_gtk_param_widget_get_int(pw, PARAM_START);
        msg.node_ind[1] =  bot_gtk_param_widget_get_int(pw, PARAM_END);
        msg.type_of_nodes = RIPL_SLAM_NODE_SELECT_T_NODES_VALID_NODE;
        msg.mode = RIPL_SLAM_NODE_SELECT_T_PUBLISH_VOXEL_MAP;

        for(int i=0; i< count; i++){
            msg.node_ind[i] = start + i;
        }
        
        ripl_slam_node_select_t_publish (self->lcm, "SLAM_3D_POINT_REQUEST", &msg);
        free(msg.node_ind);
    }
    
    //add here

    if(!strcmp(name, PARAM_SEND_MANUAL)) {
        int64_t faults=0;
        faults|= RIPL_ROBOT_STATUS_T_FAULT_MANUAL; 
        //faults|= RIPL_ROBOT_STATUS_T_FAULT_FRONT_DROPOFF;
        //faults|= RIPL_ROBOT_STATUS_T_FAULT_REAR_DROPOFF;
        ripl_robot_state_command_t state_msg;
        state_msg.utime = bot_timestamp_now();
        state_msg.sender = "viewer";
        state_msg.comment = "";
        state_msg.faults = faults;
        state_msg.fault_mask =  RIPL_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
        state_msg.state = RIPL_ROBOT_STATE_COMMAND_T_STATE_ERROR;  
        ripl_robot_state_command_t_publish (self->lcm, "ROBOT_STATE_COMMAND", &state_msg);
    }
    if(!strcmp(name, PARAM_SEND_FRONT_FAULT)) {
        int64_t faults=0;
        faults|= RIPL_ROBOT_STATUS_T_FAULT_FRONT_DROPOFF;
        //faults|= RIPL_ROBOT_STATUS_T_FAULT_REAR_DROPOFF;
        ripl_robot_state_command_t state_msg;
        state_msg.utime = bot_timestamp_now();
        state_msg.sender = "viewer";
        state_msg.comment = "";
        state_msg.faults = faults;
        state_msg.fault_mask =  RIPL_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
        state_msg.state = RIPL_ROBOT_STATE_COMMAND_T_STATE_ERROR;  
        ripl_robot_state_command_t_publish (self->lcm, "ROBOT_STATE_COMMAND", &state_msg);
    }
    if(!strcmp(name, PARAM_SEND_REAR_FAULT)) {
        int64_t faults=0;
        faults|= RIPL_ROBOT_STATUS_T_FAULT_REAR_DROPOFF;
        ripl_robot_state_command_t state_msg;
        state_msg.utime = bot_timestamp_now();
        state_msg.sender = "viewer";
        state_msg.comment = "";
        state_msg.faults = faults;
        state_msg.fault_mask =  RIPL_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
        state_msg.state = RIPL_ROBOT_STATE_COMMAND_T_STATE_ERROR;  
        ripl_robot_state_command_t_publish (self->lcm, "ROBOT_STATE_COMMAND", &state_msg);
    }
    if(!strcmp(name, PARAM_SEND_REGION)) {
        ripl_region_change_t msg;
        msg.utime = bot_timestamp_now();
        int region_id = bot_gtk_param_widget_get_int(self->pw, PARAM_REGION_ID);
        msg.region_no = region_id;
        fprintf(stderr,"New region id : %d\n", msg.region_no);
        ripl_region_change_t_publish(self->lcm, "REGION_CHANGE",&msg);
    }

    if(!strcmp(name, PARAM_SEND_FLOOR)) {
        ripl_floor_change_msg_t msg;
        msg.utime = bot_timestamp_now();
        int region_id = bot_gtk_param_widget_get_int(self->pw, PARAM_FLOOR_ID);
        msg.floor_no = region_id;
        fprintf(stderr,"New floor id : %d\n", msg.floor_no);
        ripl_floor_change_msg_t_publish(self->lcm, "FLOOR_STAUS",&msg);
    }

    if(!strcmp(name, PARAM_SEND_SEGMENT)) {
        if(self->segment_utime > 0){
            ripl_person_id_t msg;
            msg.sensor_utime = self->segment_utime;
            int person_id = bot_gtk_param_widget_get_int(self->pw, PARAM_SEGMENT_ID);
            msg.person_segment_no = person_id;
            fprintf(stderr,"Person id : %d\n", msg.person_segment_no);
            ripl_person_id_t_publish(self->lcm, "PERSON_SEGMENT_ANNOTATION",&msg);
        }
        else{
            fprintf(stderr,"Error - no time stamp heard for segments\n");
        }
    }
    
    bot_viewer_request_redraw (self->viewer);
}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
             const double ray_dir[3], const GdkEventButton *event)
{
    RendererRobotCommands *self = (RendererRobotCommands*) ehandler->user;

    /*self->dragging = 0;

    if(ehandler->picking==0){
        return 0;
    }  
    if(self->active==0){
        fprintf(stderr, "Not Active\n");
        return 0;
    }

    if(event->button != 1){
        fprintf(stderr,"Wrong Button\n");
        return 0;
        }*/

    point2d_t click_pt_local;
  
    if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
                                           POINT3D(ray_dir), 0, &click_pt_local)) {
        bot_viewer_request_redraw(self->viewer);
        //self->active = 0;
        return 0;
    }

    /*self->dragging = 1;

    self->drag_start_local = click_pt_local;
    self->drag_finish_local = click_pt_local;

    recompute_particle_distribution(self);*/

    if(bot_gtk_param_widget_get_bool(self->pw, PARAM_SEND_FOVIATION_COMMAND)){
        //fprintf(stderr,"Click Point : %f,%f\n", click_pt_local.x, click_pt_local.y);
        
        self->pos[0] = click_pt_local.x;
        self->pos[1] = click_pt_local.y;

        ripl_roi_t msg; 
        memset(&msg, 0, sizeof(ripl_roi_t)); 
        msg.utime = bot_timestamp_now();
        msg.foviation_type = RIPL_ROI_T_MOVE_HORIZONTAL; 
        msg.pos[0] = click_pt_local.x;
        msg.pos[1] = click_pt_local.y;
        msg.pos[2] = 0;
        msg.frame = RIPL_ROI_T_FRAME_LOCAL;//"local";

        ripl_roi_t_publish(self->lcm, "ROI", &msg);
    }
    /*double dx = self->drag_finish_local.x - self->drag_start_local.x;
    double dy = self->drag_finish_local.y - self->drag_start_local.y;
    */

    bot_viewer_request_redraw(self->viewer);
    return 1;
}

static void
on_segments (const lcm_recv_buf_t *rbuf, const char *channel,
             const ripl_segment_feature_list_t *msg, void *user)
{
    RendererRobotCommands *self = (RendererRobotCommands *)user;
    g_assert(self);

    fprintf(stderr, "Utime : %f\n", msg->utime / 1.0e6);
    self->segment_utime = msg->utime;

    bot_viewer_request_redraw (self->viewer);
}

static void
robot_commands_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererRobotCommands *self = (RendererRobotCommands*) renderer->user;

    glPushMatrix();
    glLineWidth(1);
    double x = self->pos[0];
    double y = self->pos[1];
    glColor3f(0, 0, 1.0);
    //fprintf(stderr,"%f,%f\n", x,y);
    glTranslatef(x, y, 0);
    bot_gl_draw_circle(0.1);
    
    glPopMatrix();
    /*RendererRobotCommands *self = (RendererRobotCommands*) renderer->user;
    if (!self->max_utime) return;

    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];

    glGetDoublev (GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev (GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

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


    // Render the current robot status
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, viewport[2], 0, viewport[3]);

    glColor3f(1,1,1);
    int8_t state = self->robot_status ? self->robot_status->state : RIPL_ROBOT_STATUS_T_STATE_UNDEFINED;
    char *robot_string;
    switch (state) 
        {
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
    bot_gl_draw_text(state_xyz, NULL, robot_string,
                     //                     GLUTIL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES |
                     BOT_GL_DRAW_TEXT_JUSTIFY_CENTER |
                     BOT_GL_DRAW_TEXT_ANCHOR_VCENTER |
                     BOT_GL_DRAW_TEXT_ANCHOR_HCENTER |
                     BOT_GL_DRAW_TEXT_DROP_SHADOW);

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
    glPopMatrix();*/

}

static void
robot_commands_free (BotRenderer *renderer) 
{
    RendererRobotCommands *self = (RendererRobotCommands*) renderer;
    
    if (self)
        free (self);
    
}

BotRenderer *renderer_robot_commands_new (BotViewer *viewer)
{
    RendererRobotCommands *self = 
        (RendererRobotCommands*) calloc (1, sizeof (RendererRobotCommands));
    self->viewer = viewer;
    self->renderer.draw = robot_commands_draw;
    self->renderer.destroy = robot_commands_free;
    self->renderer.name = "Robot Commands";
    self->renderer.user = self;
    self->renderer.enabled = 1;

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->key_press = NULL;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, 1);

    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);

    self->lcm = bot_lcm_get_global (NULL);
    
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), 
                       GTK_WIDGET(self->pw));
    gtk_widget_show (GTK_WIDGET (self->pw));

    bot_gtk_param_widget_add_double(self->pw, H_POSITION, 
                                    BOT_GTK_PARAM_WIDGET_SLIDER, -45,225 , 0.1, 0);

    bot_gtk_param_widget_add_double(self->pw, V_POSITION, 
                                    BOT_GTK_PARAM_WIDGET_SLIDER, -20,20 , 0.1, 0);
    
    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_MANUAL, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_FRONT_FAULT, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_REAR_FAULT, NULL);

    //bot_gtk_param_widget_add_int(self->pw, PARAM_START, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 1000,0.1, 0);
    //bot_gtk_param_widget_add_int(self->pw, PARAM_END, BOT_GTK_PARAM_WIDGET_SPINBOX, 0,1000, 0.1, 0);
    bot_gtk_param_widget_add_int(self->pw, PARAM_START, 
                                 BOT_GTK_PARAM_WIDGET_SLIDER, 0,1000 , 1,0);

    bot_gtk_param_widget_add_int(self->pw, PARAM_END, 
                                 BOT_GTK_PARAM_WIDGET_SLIDER, 0,1000, 1,0);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_POINT_REQUEST, NULL);

    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_SEND_FOVIATION_COMMAND , 0, NULL);
    
    bot_gtk_param_widget_add_int(self->pw, PARAM_SEGMENT_ID, BOT_GTK_PARAM_WIDGET_SPINBOX, -2, 40, 1, 0);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_SEGMENT, NULL);

    bot_gtk_param_widget_add_int(self->pw, PARAM_REGION_ID, BOT_GTK_PARAM_WIDGET_SPINBOX, -2, 40, 1, 0);

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_REGION, NULL);

    bot_gtk_param_widget_add_int(self->pw, PARAM_FLOOR_ID, BOT_GTK_PARAM_WIDGET_SPINBOX, -2, 40, 1, 0);

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_FLOOR, NULL);


    bot_gtk_param_widget_add_int(self->pw, PARAM_FLOOR_ID, BOT_GTK_PARAM_WIDGET_SPINBOX, -2, 40, 1, 0);
    
    


    ripl_segment_feature_list_t_subscribe(self->lcm, "VELODYNE_PERSON_SEGMENTS", on_segments, self);
    
    
    // translational velocity plot
    /*self->trans_vel_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->trans_vel_plot, "TV (m/s)");
    bot_gl_scrollplot2d_set_text_color   (self->trans_vel_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_bgcolor      (self->trans_vel_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_border_color (self->trans_vel_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->trans_vel_plot, 0, 1.5);
    bot_gl_scrollplot2d_add_plot    (self->trans_vel_plot, "requested", 1000);
    bot_gl_scrollplot2d_set_color   (self->trans_vel_plot, "requested", 0.7, 0, 0.7, 1);

    bot_gl_scrollplot2d_add_plot    (self->trans_vel_plot, "actual", 1000);
    bot_gl_scrollplot2d_set_color   (self->trans_vel_plot, "actual", 0, 0, 1, 1);*/
    
    return &self->renderer;
}

void setup_renderer_robot_commands (BotViewer *viewer, int priority)
{
    bot_viewer_add_renderer(viewer, 
                            renderer_robot_commands_new(viewer), priority);
}

