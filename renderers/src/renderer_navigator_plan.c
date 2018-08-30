/**
 * Navigator Plan Renderer
 *
 * Subscribes to NAV_PLAN_CHANNEL point_list message.
 *
 * Draw a line showing the trajectory.
 *
 * Widget enables turning on/off trajectory drawing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gdk/gdkkeysyms.h>

#include <lcm/lcm.h>
#include <geom_utils/geometry.h>
#include <carmen_utils/global.h>
#include "er_renderers.h"
#include "gl_utils.h"
#include <bot_core/bot_core.h>
#include <bot_vis/texture.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <hr_lcmtypes/lcm_channel_names.h>

#define RENDERER_NAME "Navigator Plan"

#define PARAM_SHOW_NAVIGATOR_PLAN "Show Navigator Plan"
#define QUERY_ELEVATOR "Query Elevator"
#define PLACE_NAV_GOAL "Place Goal"
#define START_ROBOT "Go to Goal" // Currently not enabled
#define STOP_ROBOT "Stop Robot"  // Currently not enabled
#define ADD_NEW_PLACE "Add New Place"
#define ADD_PORTAL_DOOR "Add Door"
#define ADD_PORTAL_ELEVATOR "Add Elevator"
#define QUERY_PLACE "QUERY_PLACE"
#define PARAM_FLOOR_NO "Current Floor"

typedef struct _RendererNavigatorPlan {
    BotRenderer renderer;
    BotEventHandler ehandler;

    lcm_t *lc;

    BotViewer *viewer;
    BotGtkParamWidget *pw;

    BotFrames *frames;

    ripl_point_list_t_subscription_t *navigator_plan_subscription;
    ripl_point_list_t *navigator_plan;

    gboolean param_draw_navigator_plan;
    gboolean param_goal_placement;
    gboolean param_have_goal;
    gboolean param_add_loc;
    gboolean param_add_portal;

    int portal_type; //0 for door , 1 for elevator
    float goal[2];

    int dragging;
    point2d_t drag_start_local;
    point2d_t drag_finish_local;

    int have_portal;
    point2d_t portal[2];

    point2d_t goal_mean;
    double goal_theta;
    double goal_std;

    int current_floor_ind;
    int current_floor_no;
    int sent_topo_request;
    int num_floors;

    GtkWidget *place_name_entry;
    GtkWidget *placename_dialog;
    GtkWidget *portal_name_entry;
    GtkWidget *portal_dialog;

    ripl_goal_feasibility_querry_t *goal_querry_result;
    ripl_topology_t *topology;

    GSList *portal_list;

} RendererNavigatorPlan;

void add_portal_to_list(RendererNavigatorPlan *self, ripl_portal_node_t *portal){
    /*self->portal_list.list = (ripl_portal_node_t *) realloc(self->portal_list.list,
                                                             self->portal_list.no_portals + 1);

    self->portal_list.list + self->portal_list.no_portals =
    self->portal_list.no_portals++;     */
    self->portal_list = g_slist_append(self->portal_list, ripl_portal_node_t_copy(portal));

}

static void
recompute_particle_distribution(RendererNavigatorPlan *self)
{
    self->goal_mean = self->drag_start_local;
    double dx = self->drag_finish_local.x - self->drag_start_local.x;
    double dy = self->drag_finish_local.y - self->drag_start_local.y;

    double theta = atan2(dy,dx);
    self->goal_theta = theta;

    self->goal_std = sqrt(dx*dx + dy*dy);

}

void check_current_floor(RendererNavigatorPlan *s){
    //send a floor status update message
    ripl_map_request_msg_t msg;
    msg.utime = bot_timestamp_now();
    msg.requesting_prog = "NAV_RENDERER";
    ripl_map_request_msg_t_publish(s->lc, "FLOOR_STATUS_REQUEST", &msg);
}


static
void add_place_button(GtkWidget *button __attribute__ ((unused)),
                      gpointer user_data __attribute__ ((unused)))
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user_data;
    char *errs;
    char name[100];
    strcpy(name, gtk_entry_get_text(GTK_ENTRY(self->place_name_entry)));

    fprintf(stderr,"\nPlace Name : %s (%f,%f)\n",name,self->goal_mean.x,self->goal_mean.y);


    /*ripl_tagged_node_t msg;
    msg.utime = bot_timestamp_now();
    msg.label = strdup(name);
    msg.type = "";
    msg.x = self->goal_mean.x;
    msg.y = self->goal_mean.y;
    msg.theta = self->goal_theta;
    msg.floor_index = self->current_floor_ind;
    msg.floor_no = self->current_floor_no;
    msg.dir = "";
    msg.view = "";
    msg.pos = "";
    msg.prop = "";
    msg.pron = "";

    ripl_tagged_node_t_publish(self->lc, "ADD_TAGGED_PLACE", &msg);
    free(msg.label);*/

    ripl_place_node_t msg;
    msg.name = strdup(name);
    msg.type = " ";
    msg.x = self->goal_mean.x;//x;
    msg.y = self->goal_mean.y;//y;
    msg.theta = self->goal_theta;//0;
    msg.floor_ind = self->current_floor_ind;
    msg.floor_no = self->current_floor_no;
    msg.std = self->goal_std;
    ripl_place_node_t_publish(self->lc, "ADD_PLACE_NODE", &msg);

    free(msg.name);

    self->param_add_loc = 0;
    gtk_widget_destroy(self->placename_dialog);
}

void start_add_placename(double x, double y, double theta, double std,
                         RendererNavigatorPlan *self)
{
    static GtkWidget *name_label, *x_label, *y_label, *theta_label, *x_entry, *y_entry;
    static GtkWidget *theta_entry, *x_std_entry,
        *y_std_entry, *theta_std_entry;
    int edit_place_id;

    GtkWidget *hbox, *label, *button;
    char buffer[10];

    self->placename_dialog = gtk_dialog_new();
    hbox = gtk_hbox_new(FALSE, 0);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (self->placename_dialog)->vbox),
                        hbox, TRUE, TRUE, 0);
    name_label = gtk_label_new("Place name: ");
    gtk_box_pack_start (GTK_BOX (hbox), name_label, TRUE, TRUE, 0);
    self->place_name_entry = gtk_entry_new_with_max_length(21);
    gtk_widget_set_usize(self->place_name_entry, 90, 20);
    gtk_box_pack_start (GTK_BOX(hbox), self->place_name_entry, TRUE, TRUE, 0);

    hbox = gtk_hbox_new(FALSE, 3);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (self->placename_dialog)->vbox),
                        hbox, TRUE, TRUE, 0);

    x_label = gtk_label_new("X: ");
    gtk_box_pack_start (GTK_BOX (hbox), x_label, TRUE, TRUE, 0);
    x_entry = gtk_entry_new_with_max_length(5);
    gtk_widget_set_usize(x_entry, 45, 20);
    gtk_box_pack_start (GTK_BOX (hbox), x_entry, TRUE, TRUE, 0);
    sprintf(buffer, "%.2f", x);
    gtk_entry_set_text(GTK_ENTRY(x_entry), buffer);

    y_label = gtk_label_new("Y: ");
    gtk_box_pack_start (GTK_BOX (hbox), y_label, TRUE, TRUE, 0);
    y_entry = gtk_entry_new_with_max_length(5);
    gtk_widget_set_usize(y_entry, 45, 20);
    gtk_box_pack_start (GTK_BOX (hbox), y_entry, TRUE, TRUE, 0);
    sprintf(buffer, "%.2f", y);
    gtk_entry_set_text(GTK_ENTRY(y_entry), buffer);

    theta_label = gtk_label_new("Theta (deg): ");
    gtk_box_pack_start (GTK_BOX (hbox), theta_label, TRUE, TRUE, 0);
    theta_entry = gtk_entry_new_with_max_length(5);
    gtk_widget_set_usize(theta_entry, 45, 20);
    gtk_box_pack_start (GTK_BOX (hbox), theta_entry, TRUE, TRUE, 0);

    sprintf(buffer, "%.2f", theta);
    gtk_entry_set_text(GTK_ENTRY(theta_entry), buffer);

    hbox = gtk_hbox_new(FALSE, 3);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (self->placename_dialog)->vbox),
                        hbox, TRUE, TRUE, 0);

    label = gtk_label_new("Tollerance : ");
    gtk_box_pack_start (GTK_BOX (hbox), label, TRUE, TRUE, 0);
    x_std_entry = gtk_entry_new_with_max_length(5);
    gtk_widget_set_usize(x_std_entry, 45, 20);
    gtk_box_pack_start (GTK_BOX (hbox), x_std_entry, TRUE, TRUE, 0);

    sprintf(buffer, "%.2f", std);
    gtk_entry_set_text(GTK_ENTRY(x_std_entry), buffer);

    hbox = GTK_DIALOG(self->placename_dialog)->action_area;

    button = gtk_button_new_with_label("OK");
    gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

    gtk_signal_connect(GTK_OBJECT(button), "clicked",
                       (GtkSignalFunc)add_place_button, self);

    button = gtk_button_new_with_label("Cancel");
    gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

    gtk_signal_connect_object(GTK_OBJECT(button),
                              "clicked", (GtkSignalFunc)gtk_widget_destroy,
                              (gpointer)self->placename_dialog);

    edit_place_id = -1;
    gtk_widget_show_all(self->placename_dialog);
}


static
void add_portal_button(GtkWidget *button __attribute__ ((unused)),
                      gpointer user_data __attribute__ ((unused)))
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user_data;
    char *errs;
    char name[100];
    //strcpy(name, gtk_entry_get_text(GTK_ENTRY(self->portal_name_entry)));

    fprintf(stderr,"\nPortal Type : %d Loc (%f,%f) => Loc (%f,%f)\n",
            self->portal_type, self->drag_start_local.x,
            self->drag_start_local.y, self->drag_finish_local.x,
            self->drag_finish_local.y);

    ripl_portal_node_t msg;

    if(self->portal_type == 0){
        fprintf(stderr, "Type : Door\n");
        msg.type = ERLCM_PORTAL_NODE_T_PORTAL_DOOR;
    }
    //elevators need to be treated differently - they span multiple floors
    else if(self->portal_type == 1){
        fprintf(stderr, "Type : Elevator\n");
        msg.type = ERLCM_PORTAL_NODE_T_PORTAL_ELEVATOR;
    }

    msg.node_id = 0; //will not be used here - should be reprocessed somewhere else
    msg.xy0[0] = self->drag_start_local.x;
    msg.xy0[1] = self->drag_start_local.y;
    msg.xy1[0] = self->drag_finish_local.x;
    msg.xy1[1] = self->drag_finish_local.y;

    //this is a bit tricky - we need to say what floors this elevator spans
    //maybe we can amalgamate it at the receiver end
    msg.floor_ind = self->current_floor_ind;
    msg.floor_no = self->current_floor_no;
    msg.create_utime = bot_timestamp_now();

    ripl_portal_node_t_publish(self->lc, "ADD_PORTAL_NODE" , &msg);

    add_portal_to_list(self, &msg);

    gtk_widget_destroy(self->portal_dialog);
}


void start_add_portal(RendererNavigatorPlan *self)
{
    static GtkWidget *name_label, *xy1_label, *xy2_label, *xy1_entry, *xy2_entry;
    int edit_place_id;

    GtkWidget *hbox, *label, *button;
    char buffer[20];

    self->portal_dialog = gtk_dialog_new();
    hbox = gtk_hbox_new(FALSE, 0);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (self->portal_dialog)->vbox),
                        hbox, TRUE, TRUE, 0);
    if(self->portal_type == 0){
        name_label = gtk_label_new("Door");
    }
    else if(self->portal_type == 1){
        name_label = gtk_label_new("Elevator");
    }

    gtk_box_pack_start (GTK_BOX (hbox), name_label, TRUE, TRUE, 0);
    self->portal_name_entry = gtk_entry_new_with_max_length(21);

    hbox = gtk_hbox_new(FALSE, 3);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (self->portal_dialog)->vbox),
                        hbox, TRUE, TRUE, 0);

    xy1_label = gtk_label_new("XY (1) : ");
    gtk_box_pack_start (GTK_BOX (hbox), xy1_label, TRUE, TRUE, 0);
    xy1_entry = gtk_entry_new_with_max_length(20);
    gtk_widget_set_usize(xy1_entry, 100, 20);
    gtk_box_pack_start (GTK_BOX (hbox), xy1_entry, TRUE, TRUE, 0);
    sprintf(buffer, "%.2f, %.2f", self->drag_start_local.x, self->drag_start_local.y);
    gtk_entry_set_text(GTK_ENTRY(xy1_entry), buffer);

    xy2_label = gtk_label_new("XY (2) : ");
    gtk_box_pack_start (GTK_BOX (hbox), xy2_label, TRUE, TRUE, 0);
    xy2_entry = gtk_entry_new_with_max_length(20);
    gtk_widget_set_usize(xy2_entry, 100, 20);
    gtk_box_pack_start (GTK_BOX (hbox), xy2_entry, TRUE, TRUE, 0);
    sprintf(buffer, "%.2f, %.2f", self->drag_finish_local.x, self->drag_finish_local.y);
    gtk_entry_set_text(GTK_ENTRY(xy2_entry), buffer);

    hbox = GTK_DIALOG(self->portal_dialog)->action_area;

    button = gtk_button_new_with_label("OK");
    gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

    gtk_signal_connect(GTK_OBJECT(button), "clicked",
                       (GtkSignalFunc)add_portal_button, self);

    button = gtk_button_new_with_label("Cancel");
    gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

    gtk_signal_connect_object(GTK_OBJECT(button),
                              "clicked", (GtkSignalFunc)gtk_widget_destroy,
                              (gpointer)self->portal_dialog);

    edit_place_id = -1;
    gtk_widget_show_all(self->portal_dialog);
}


static void navigator_plan_handler(const lcm_recv_buf_t *rbuf, const char *channel, const ripl_point_list_t *msg,
                                   void *user)
{
    fprintf(stderr, "N");

    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user;
    if (self->navigator_plan != NULL) {
        ripl_point_list_t_destroy(self->navigator_plan);
    }
    self->navigator_plan = ripl_point_list_t_copy(msg);

    bot_viewer_request_redraw(self->viewer);
}

static void  goal_querry_result_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)), const ripl_goal_feasibility_querry_t * msg, void * user  __attribute__((unused)))
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user;
    if(self->goal_querry_result != NULL){
        ripl_goal_feasibility_querry_t_destroy(self->goal_querry_result);
    }
    self->goal_querry_result = ripl_goal_feasibility_querry_t_copy(msg);

    bot_viewer_request_redraw(self->viewer);
}

static void floor_status_handler(const lcm_recv_buf_t *rbuf,
                                 const char *channel,
                                 const ripl_floor_status_msg_t *msg,
                                 void *user)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user;

    self->current_floor_ind = msg->floor_ind;
    self->current_floor_no = msg->floor_no;
}

static void topology_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                             const ripl_topology_t *msg, void *user)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user;

    if(self->topology){
        ripl_topology_t_destroy(self->topology);
    }
    self->topology = ripl_topology_t_copy(msg);

}


static void multi_gridmap_handler(const lcm_recv_buf_t *rbuf,
                                  const char *channel,
                                  const ripl_multi_gridmap_t *msg,
                                  void *user)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan *) user;
    if (self->num_floors != msg->no_floors) {

        // Update the gtk widget to respect the new floors
        if (msg->no_floors > 0)
            bot_gtk_param_widget_clear_enum (self->pw, PARAM_FLOOR_NO);

        char name[26];
        for (int i=0; i < msg->no_floors; i++) {
            sprintf (name, "%d", msg->maps[i].floor_no);

            bot_gtk_param_widget_modify_enum (self->pw, PARAM_FLOOR_NO, name, msg->maps[i].floor_no);
        }
        bot_gtk_param_widget_set_enum (self->pw, PARAM_FLOOR_NO,
                                       msg->current_floor_ind);

        self->num_floors = msg->no_floors;
    }

    bot_viewer_request_redraw(self->viewer);
}



void activate_nav(RendererNavigatorPlan *self,int type)
{
    //self->active = 1;
    if(type==1){
        bot_viewer_set_status_bar_message(self->viewer,
                                          "Click to Place the goal");
    }else if (type==2){//Start robot
        bot_viewer_set_status_bar_message(self->viewer,
                                          "Started Robot");
    }else if (type==3){//Start robot
        bot_viewer_set_status_bar_message(self->viewer,
                                          "Stopped Robot");
    }
    if(type==4){
        bot_viewer_set_status_bar_message(self->viewer,
                                          "Click to Place new location");
    }
    if(type==5){
        bot_viewer_set_status_bar_message(self->viewer,
                                          "Click to Place Portal");
    }
}

static void navigator_plan_renderer_free(BotRenderer *super)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) super->user;
    if (self->navigator_plan) {
        ripl_point_list_t_destroy(self->navigator_plan);
    }
    free(self);
}

void draw_filled_circle(float x,float y,float radius)
{
    glColor4f(1,0,0,.6);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y);
    for (int i=0; i < 360; i++){
        float degInRad = i*M_PI/180;
        glVertex2f(x+sin(degInRad)*radius,y+cos(degInRad)*radius);
    }
    glEnd();
}

void draw_circle(float x,float y,float radius)
{
    glColor4f(0,0,0,.6);
    glBegin(GL_LINE_LOOP);

    for (int i=0; i < 360; i++)
        {
            float degInRad = i*M_PI/180;
            glVertex2f(x+cos(degInRad)*radius,y+sin(degInRad)*radius);
        }
    glEnd();
}




static void navigator_plan_renderer_draw(BotViewer *viewer, BotRenderer *super)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) super->user;

    if(!self->sent_topo_request){
        //send topo request
        ripl_map_request_msg_t msg;
        msg.utime = bot_timestamp_now();
        msg.requesting_prog = "TOPO_NAV";
        ripl_map_request_msg_t_publish(self->lc, "TOPOLOGY_REQUEST", &msg);
        self->sent_topo_request = 1;
    }

    // draw navigator plan if we have it and widget box is selected
    /*if (self->navigator_plan && self->param_draw_navigator_plan) {

      glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
      glLineWidth(2);
      glBegin(GL_LINE_STRIP);
      glColor3d(0.8, 0.2, 0.2);
      for (int i = 0; i < self->navigator_plan->num_points; i++) {
      glVertex3dv(point3d_as_array(&self->navigator_plan->points[i]));
      }
      glEnd();
      glPopAttrib();
      }
      if(self->param_have_goal){
      draw_filled_circle(self->goal[0],self->goal[1],0.2);
      }*/

    glEnable (GL_LIGHTING);
    //glEnable (GL_DEPTH_TEST);
    glDisable (GL_DEPTH_TEST);
    glEnable (GL_LINE_SMOOTH);
    glShadeModel (GL_SMOOTH);

    glLineWidth (2.0);
    float color_curr[3];

    color_curr[0] = 0.0;
    color_curr[1] = 0.0;
    color_curr[2] = 1.0;

    if(self->goal_querry_result){
        //go through the list and draw the portals and the goal -- what to do about the
        //goal on the other floor ?? - ignore this and draw anyway

        //glBegin(GL_LINE_STRIP);

        for(int i=0; i < self->goal_querry_result->portal_list.no_of_portals; i++){
            //fprintf(stderr, " Portal : %d\n" , i);

            glPushMatrix();

            double x0 = self->goal_querry_result->portal_list.portals[i].xy0[0];
            double y0 = self->goal_querry_result->portal_list.portals[i].xy0[1];

            glTranslatef(x0, y0, 0);
            bot_gl_draw_circle(0.5);
            glPopMatrix();
            //glVertex2f(x0,y0);

            glPushMatrix();


            double x1 = self->goal_querry_result->portal_list.portals[i].xy1[0];
            double y1 = self->goal_querry_result->portal_list.portals[i].xy1[1];

            glTranslatef(x1, y1, 0);
            bot_gl_draw_circle(0.5);
            glPopMatrix();
            //            glVertex2f(x1,y1);
        }

        //glEnd();

        for(int i=0; i < self->goal_querry_result->portal_list.no_of_portals; i++){
            double x0 = self->goal_querry_result->portal_list.portals[i].xy0[0];
            double y0 = self->goal_querry_result->portal_list.portals[i].xy0[1];

            double x1 = self->goal_querry_result->portal_list.portals[i].xy1[0];
            double y1 = self->goal_querry_result->portal_list.portals[i].xy1[1];

            double pos[3] = {(x0+x1)/2, (y0+y1)/2, 0};
            char buf[20];
            sprintf(buf, "Node Order : %d",i);
            bot_gl_draw_text(pos, NULL, buf, 0);
        }
    }

    if(self->topology){
        for(int i=0; i < self->topology->portal_list.no_of_portals; i++){
            ripl_portal_node_t *c_portal = &self->topology->portal_list.portals[i];

            if(c_portal->floor_ind != self->current_floor_ind){
                continue;
            }

            if(c_portal->type ==  ERLCM_PORTAL_NODE_T_PORTAL_DOOR){
                color_curr[0] = 0.0;
                color_curr[1] = 0.0;
                color_curr[2] = 1.0;
            }
            else if(c_portal->type ==  ERLCM_PORTAL_NODE_T_PORTAL_ELEVATOR){
                color_curr[0] = 1.0;
                color_curr[1] = 0.0;
                color_curr[2] = 0.0;
            }
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
            glBegin(GL_LINE_STRIP);
            glVertex2f(c_portal->xy0[0],c_portal->xy0[1]);
            glVertex2f(c_portal->xy1[0],c_portal->xy1[1]);
            glEnd();

            // Convert from global to local frame
            double pos_global[3] = {(c_portal->xy0[0] + c_portal->xy1[0])/2 ,(c_portal->xy0[1] + c_portal->xy1[1])/2,0};
            double pos_local[3];

            bot_frames_transform_vec (self->frames, "global", "local", pos_global, pos_local);

            //bot_frames_transform_vec (

            if(c_portal->type == 0){
                bot_gl_draw_text(pos_local, NULL, "Door", 0);
            }
            else if(c_portal->type == 1){
                bot_gl_draw_text(pos_local, NULL, "Elevator", 0);
            }
            glLineWidth (3.0);
            glPushMatrix();
            glTranslatef(pos_local[0],pos_local[1],0);
            bot_gl_draw_circle(0.3);
            glPopMatrix();


            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

            glPushMatrix();
            glTranslatef(c_portal->xy1[0],c_portal->xy1[1],0);
            bot_gl_draw_circle(0.3);
            glPopMatrix();
            glLineWidth (2.0);
        }

        for(int i=0; i < self->topology->place_list.place_count; i++){
            ripl_place_node_t *place = &self->topology->place_list.trajectory[i];


            if(place->floor_ind != self->current_floor_ind){
                continue;
            }

            double pos_global[3] = {place->x, place->y, 0};
            double pos_local[3];

            bot_frames_transform_vec (self->frames, "global", "local", pos_global, pos_local);
            BotTrans global_to_local;
            bot_frames_get_trans (self->frames, "global", "local", &global_to_local);
            double rpy_global[3] = {0, 0, place->theta};
            double quat_global[4], quat_local[4];
            double rpy_local[3];

            bot_roll_pitch_yaw_to_quat (rpy_global, quat_global);
            bot_quat_mult (quat_local, global_to_local.rot_quat, quat_global);
            bot_quat_to_roll_pitch_yaw (quat_local, rpy_local);


            color_curr[0] = 0.0;
            color_curr[1] = 1.0;
            color_curr[2] = 0.0;

            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

            glLineWidth (3.0);
            glPushMatrix();
            glTranslatef(pos_local[0],pos_local[1],0);
            bot_gl_draw_circle(0.3);

            color_curr[0] = 0.0;
            color_curr[1] = 0.0;
            color_curr[2] = 0.0;
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
            double pos[3] = {0,0,0};
            bot_gl_draw_text(pos, NULL, place->name, 0);
            color_curr[0] = 1.0;
            color_curr[1] = 0.0;
            color_curr[2] = 0.0;
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

            glBegin(GL_LINE_STRIP);
            glVertex2f(0.0,0.0);
            glVertex2f(place->std*cos(rpy_local[2]), place->std*sin(rpy_local[2]));
            glEnd();

            glPopMatrix();

            glLineWidth (2.0);
        }
    }

    if(self->portal_list){
        GSList *curr_portal = self->portal_list;
        while(curr_portal){
            ripl_portal_node_t *c_portal = (ripl_portal_node_t *) curr_portal->data;
            if(c_portal->type ==  ERLCM_PORTAL_NODE_T_PORTAL_DOOR){
                color_curr[0] = 0.0;
                color_curr[1] = 0.0;
                color_curr[2] = 1.0;
            }
            else if(c_portal->type ==  ERLCM_PORTAL_NODE_T_PORTAL_ELEVATOR){
                color_curr[0] = 1.0;
                color_curr[1] = 0.0;
                color_curr[2] = 0.0;
            }

            // Convert from global to local frame
            double pos0_global[3] = {c_portal->xy0[0], c_portal->xy0[1], 0};
            double pos0_local[3];

            bot_frames_transform_vec (self->frames, "global", "local", pos0_global, pos0_local);

            double pos1_global[3] = {c_portal->xy1[0], c_portal->xy1[1], 0};
            double pos1_local[3];

            bot_frames_transform_vec (self->frames, "global", "local", pos1_global, pos1_local);

            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
            glBegin(GL_LINE_STRIP);
            glVertex2f(pos0_local[0],pos0_local[1]);
            glVertex2f(pos1_local[0],pos1_local[1]);
            glEnd();

            glLineWidth (3.0);
            glPushMatrix();
            glTranslatef(c_portal->xy0[0],c_portal->xy0[1],0);
            bot_gl_draw_circle(0.3);
            glPopMatrix();


            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

            glPushMatrix();
            glTranslatef(pos1_local[0], pos1_local[1],0);
            bot_gl_draw_circle(0.3);
            glPopMatrix();
            glLineWidth (2.0);
            curr_portal = g_slist_next(curr_portal);
        }
    }


    if(self->param_add_portal){
         color_curr[0] = 1.0;
         color_curr[1] = 0.0;
         color_curr[2] = 0.0;
         glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
         glBegin(GL_LINE_STRIP);
         glVertex2f(self->drag_start_local.x, self->drag_start_local.y);
         glVertex2f(self->drag_finish_local.x, self->drag_finish_local.y);
         glEnd();

         glLineWidth (3.0);
         glPushMatrix();
         glTranslatef(self->drag_start_local.x, self->drag_start_local.y,0);
         bot_gl_draw_circle(0.3);
         glPopMatrix();

         color_curr[0] = 0.0;
         color_curr[1] = 0.0;
         color_curr[2] = 1.0;
         glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

         glPushMatrix();
         glTranslatef(self->drag_finish_local.x, self->drag_finish_local.y,0);
         bot_gl_draw_circle(0.3);
         glPopMatrix();
         glLineWidth (2.0);

    }


    /*else if(self->have_portal){


        color_curr[0] = 1.0;
        color_curr[1] = 0.0;
        color_curr[2] = 0.0;
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
        glBegin(GL_LINE_STRIP);
        glVertex2f(self->portal[0].x, self->portal[0].y);
        glVertex2f(self->portal[1].x, self->portal[1].y);
        glEnd();

        glLineWidth (3.0);
        glPushMatrix();
        glTranslatef(self->portal[0].x, self->portal[0].y,0);
        bot_gl_draw_circle(0.3);
        glPopMatrix();

        color_curr[0] = 0.0;
        color_curr[1] = 0.0;
        color_curr[2] = 1.0;
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

        glPushMatrix();
        glTranslatef(self->portal[1].x, self->portal[1].y,0);
        bot_gl_draw_circle(0.3);
        glPopMatrix();
        glLineWidth (2.0);

        }*/

    else{

        if(self->param_add_loc){
            color_curr[0] = 1.0;
            color_curr[1] = 0.0;
            color_curr[2] = 0.0;
        }
        else{
            color_curr[0] = 0.0;
            color_curr[1] = 1.0;
            color_curr[2] = 0.0;
        }
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

        //draw the new goal region
        glTranslatef(self->goal_mean.x, self->goal_mean.y, 0);
        bot_gl_draw_circle(self->goal_std);

        glBegin(GL_LINE_STRIP);
        glVertex2f(0.0,0.0);

        glVertex2f(self->goal_std*cos(self->goal_theta),self->goal_std*sin(self->goal_theta));

        glEnd();

        glTranslatef(-self->goal_mean.x, -self->goal_mean.y, 0);
    }
    if (self->navigator_plan && self->param_draw_navigator_plan) {

        glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);
        glColor3d(0.8, 0.2, 0.2);
        for (int i = 0; i < self->navigator_plan->num_points; i++) {
            glVertex3dv(point3d_as_array(&self->navigator_plan->points[i]));
        }
        glEnd();
        glPopAttrib();
    }



}

/* void publish_mission_control(lcm_t *lc,int type){//ripl_mission_control_type_t type){ */
/*     ripl_mission_control_msg_t msg; */
/*     msg.utime = bot_timestamp_now(); */
/*     msg.type = type; */
/*     ripl_mission_control_msg_t_publish(lc, MISSION_CONTROL_CHANNEL, &msg); */
/* } */

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name,
                                    void *user)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) user;

    self->param_draw_navigator_plan = bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_NAVIGATOR_PLAN);

    if(!strcmp(name, PLACE_NAV_GOAL)) {
        activate_nav(self,1);
        self->param_goal_placement = 1;
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        //fprintf(stderr,"Nav Goal\n");
    }
    else if(!strcmp(name,QUERY_ELEVATOR)){
        ripl_goal_t ele_msg;
        memset(&ele_msg, 0, sizeof(ripl_goal_t));
        ele_msg.pos[0] = 28.563433723667067;
        ele_msg.pos[1] = 14.270711814186335;
        ripl_goal_t_publish(self->lc, "CHECK_ELEVATOR", &ele_msg);
        fprintf(stderr,"Checking for elevator\n");
    }
    else if(!strcmp(name, START_ROBOT)){
        activate_nav(self,2);
        //start navigator to issue commands to the robot
        // NOBODY SUBSCRIBES TO THIS MESSAGE
        //publish_mission_control(self->lc, ERLCM_MISSION_CONTROL_MSG_T_NAVIGATOR_GO);
        //publish speech command go message
        ripl_speech_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.cmd_type = "FOLLOWER";
        msg.cmd_property = "GO";
        ripl_speech_cmd_t_publish(self->lc, "WAYPOINT_NAVIGATOR", &msg);


        //fprintf(stderr,"Start Robot\n");
    }
    else if(!strcmp(name, STOP_ROBOT)) {
        activate_nav(self,3);//pause the navigator
        // NOBODY SUBSCRIBES TO THIS MESSAGE
        //publish_mission_control(self->lc, ERLCM_MISSION_CONTROL_MSG_T_NAVIGATOR_PAUSE);
        ripl_speech_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.cmd_type = "FOLLOWER";
        msg.cmd_property = "STOP";
        ripl_speech_cmd_t_publish(self->lc, "WAYPOINT_NAVIGATOR", &msg);
        //fprintf(stderr,"Stop Robot\n");
    }
    else if(!strcmp(name, ADD_NEW_PLACE)) {
        self->param_add_loc = 1;
        activate_nav(self,4);
        //self->param_place_addition = 1;
        //pause the navigator
        // NOBODY_SUBSCRIBES_TO_THIS MESSAGE
        //publish_mission_control(self->lc, CARMEN3D_MISSION_NAVIGATOR_PAUSE);
        fprintf(stderr,"Adding New Place : %d\n", self->param_add_loc);

        //ask for floor
        check_current_floor(self);
    }
    else if(!strcmp(name, ADD_PORTAL_DOOR)) {
        self->param_add_portal = 1;
        activate_nav(self,5);
        fprintf(stderr,"Adding New Portal Door : %d\n", self->param_add_portal);
        self->portal_type = 0;
        //ask for floor
        check_current_floor(self);

    }

    else if(!strcmp(name, ADD_PORTAL_ELEVATOR)) {
        self->param_add_portal = 1;
        activate_nav(self,5);
        fprintf(stderr,"Adding New Portal Elevator: %d\n", self->param_add_portal);
        self->portal_type = 1;
        //ask for floor
        check_current_floor(self);

    }
    else if(!strcmp(name, QUERY_PLACE)){
        //send querry msg - fixed query for now
        ripl_place_node_t msg;
        msg.name = "lounge";
        msg.type = " ";
        msg.x = 0;
        msg.y = 0;
        msg.theta = 0;
        //this should be filled with the proper floor
        msg.floor_ind = self->current_floor_ind;
        msg.floor_no = self->current_floor_no;
        msg.std = 0;
        ripl_place_node_t_publish(self->lc, "GOAL_NODE", &msg);
    }
    else if(!strcmp(name,PARAM_FLOOR_NO)){
        int floor_no = bot_gtk_param_widget_get_enum(self->pw, PARAM_FLOOR_NO);
        ripl_floor_change_msg_t msg;
        msg.utime = bot_timestamp_now();
        msg.floor_no = floor_no;
        fprintf (stdout, "Navigator Renderer: Publishing floor disabled\n");
        //ripl_floor_change_msg_t_publish(self->lc, "FLOOR_CHANGE",&msg);
    }

    bot_viewer_request_redraw(self->viewer);
}


static int key_press (BotViewer *viewer, BotEventHandler *ehandler,
                      const GdkEventKey *event)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) ehandler->user;

    if (self->param_goal_placement && event->keyval == GDK_Escape) {
        ehandler->picking = 0;
        self->param_goal_placement = 0;

        bot_gtk_param_widget_set_enabled (self->pw, PLACE_NAV_GOAL, TRUE);

        return 1;
    }

    return 0;
}


static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3],
                         const GdkEventMotion *event)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) ehandler->user;

    // Is this mouse_motion meant for us
    if(!self->dragging || (self->param_goal_placement == 0 && self->param_add_loc ==0 && self->param_add_portal == 0) )
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

static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3],
                         const double ray_dir[3], const GdkEventButton *event)
{

    RendererNavigatorPlan *self = (RendererNavigatorPlan*) ehandler->user;

    // Was the mouse_release event meant for us.
    if(self->param_goal_placement == 1){

        if (self->dragging) {
            self->dragging = 0;
        }

        self->param_goal_placement = 0;

        self->param_have_goal = 1;
        self->goal[0] = self->goal_mean.x;
        self->goal[1] = self->goal_mean.y;

        ripl_point_t goal;
        memset(&goal, 0, sizeof(carmen_point_t));
        goal.x = self->goal_mean.x;
        goal.y = self->goal_mean.y;
        //goal.z = 0.5; //doesnt matter
        goal.yaw = self->goal_theta;
        fprintf(stderr,"Goal Yaw : %f\n", self->goal_theta);
        ripl_navigator_goal_msg_t msg;
        msg.goal = goal;
        msg.use_theta = 1;//0;
        msg.utime = bot_timestamp_now();
        msg.nonce = random();
        msg.sender = ERLCM_NAVIGATOR_GOAL_MSG_T_SENDER_WAYPOINT_TOOL;
        ripl_navigator_goal_msg_t_publish(self->lc, "NAV_GOAL_LOCAL", &msg);

        // Comment in navigator3d.c that floor number is ignored
        //ripl_navigator_floor_goal_msg_t msg;
        //msg.goal_msg.goal = goal;
        //msg.goal_msg.use_theta = 1;//0;
        //msg.goal_msg.utime = bot_timestamp_now();
        //msg.goal_msg.nonce = random();
        //msg.goal_msg.sender = ERLCM_NAVIGATOR_GOAL_MSG_T_SENDER_WAYPOINT_TOOL;
        //msg.floor_no = 2;
        //ripl_navigator_floor_goal_msg_t_publish(self->lc, "NAV_GOAL_FLOOR", &msg);

        fprintf(stderr, "Goal: x %f y %f z %f yaw %f\n", goal.x, goal.y, goal.z, goal.yaw);

        bot_viewer_set_status_bar_message(self->viewer,"Current Goal is at (%f,%f)",self->goal[0],self->goal[1]);
        bot_viewer_request_redraw (self->viewer);

        ehandler->picking = 0;
        return 1;
    }

    // Was the mouse_release event meant for us.
    if(self->param_add_loc== 1){

        if (self->dragging) {
            self->dragging = 0;
        }

        self->param_add_loc = 0;

        self->param_have_goal = 1;
        self->goal[0] = self->goal_mean.x;
        self->goal[1] = self->goal_mean.y;


        start_add_placename(self->goal_mean.x, self->goal_mean.y, self->goal_theta,
                            self->goal_std, self);


        bot_viewer_set_status_bar_message(self->viewer,"New Place is at (%f,%f)",self->goal[0],self->goal[1]);
        bot_viewer_request_redraw (self->viewer);

        ehandler->picking = 0;
        return 1;
    }

    // Was the mouse_release event meant for us.
    if(self->param_add_portal== 1){
        self->param_add_portal = 0;

        self->have_portal = 1;

        if (self->dragging) {
            self->dragging = 0;
        }

        self->portal[0] = self->drag_start_local;
        self->portal[1] = self->drag_finish_local;

        //self->portal_type

        start_add_portal(self);

        //start_add_placename(self->goal_mean.x, self->goal_mean.y, self->goal_theta,
        //self->goal_std, self);


        bot_viewer_set_status_bar_message(self->viewer,"New Portal is at (%f,%f) => (%f,%f)",self->drag_start_local.x, self->drag_start_local.y, self->drag_finish_local.x, self->drag_finish_local.y);
        bot_viewer_request_redraw (self->viewer);

        ehandler->picking = 0;
        return 1;
    }

    ehandler->picking = 0;

    // If we've gotten here, the event wasn't meant for us.
    return 0;
}
static int mouse_press(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3],
                       const double ray_dir[3], const GdkEventButton *event)
{

    RendererNavigatorPlan *self = (RendererNavigatorPlan*) ehandler->user;

    self->dragging = 0;

    // Is this event meant for us?
    if (event->button==1 && self->param_goal_placement==1 && ehandler->picking) {

        point2d_t click_pt_local;

        if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
                                               POINT3D(ray_dir), 0, &click_pt_local)) {
            bot_viewer_request_redraw(self->viewer);
            self->param_goal_placement = 0;
            return 0;
        }

        self->dragging = 1;

        self->drag_start_local = click_pt_local;
        self->drag_finish_local = click_pt_local;

        recompute_particle_distribution(self);

        bot_viewer_request_redraw(self->viewer);
        return 1;
    }
    else if (self->param_add_loc==1){// && ehandler->picking) {

        point2d_t click_pt_local;

        if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
                                               POINT3D(ray_dir), 0, &click_pt_local)) {
            bot_viewer_request_redraw(self->viewer);
            self->param_goal_placement = 0;
            return 0;
        }

        self->dragging = 1;

        self->drag_start_local = click_pt_local;
        self->drag_finish_local = click_pt_local;

        recompute_particle_distribution(self);

        bot_viewer_request_redraw(self->viewer);
        return 1;
    }

    else if (self->param_add_portal==1){
        self->have_portal = 0;

        self->dragging = 1;

        point2d_t click_pt_local;

        if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
                                               POINT3D(ray_dir), 0, &click_pt_local)) {
            bot_viewer_request_redraw(self->viewer);
            self->param_goal_placement = 0;
            return 0;
        }

        self->drag_start_local = click_pt_local;
        self->drag_finish_local = click_pt_local;

        //recompute_particle_distribution(self);

        bot_viewer_request_redraw(self->viewer);
        return 1;
    }

    return 0;
}


static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile,
                                void *user_data)
{
    RendererNavigatorPlan *self = user_data;
    bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile,
                                void *user_data)
{
    RendererNavigatorPlan *self = user_data;
    bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

void setup_renderer_navigator_plan(BotViewer *viewer, int render_priority, 
                                       lcm_t *lcm, BotFrames *frames)
{
    RendererNavigatorPlan *self = (RendererNavigatorPlan*) calloc(1, sizeof(RendererNavigatorPlan));
    BotRenderer *renderer = &self->renderer;

    renderer->draw = navigator_plan_renderer_draw;
    renderer->destroy = navigator_plan_renderer_free;
    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;


    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->key_press = key_press;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->user = self;

    self->viewer = viewer;
    self->lc = lcm;//globals_get_lcm_full(NULL,1);
    self->frames = frames;

    // message subscriptions
    self->navigator_plan_subscription = ripl_point_list_t_subscribe(self->lc, NAV_PLAN_CHANNEL,
                                                                     navigator_plan_handler, self);

    ripl_floor_status_msg_t_subscribe(self->lc,"CURRENT_FLOOR_STATUS",
                                       floor_status_handler,
                                       self);

    ripl_goal_feasibility_querry_t_subscribe(self->lc,
                                              "GOAL_FEASIBILITY_RESULT",
                                              goal_querry_result_handler,
                                              self);

    ripl_topology_t_subscribe(self->lc,"MAP_SERVER_TOPOLOGY",
                               topology_handler,
                               self);

    ripl_topology_t_subscribe(self->lc,"TOPOLOGY",
                               topology_handler,
                               self);

    ripl_multi_gridmap_t_subscribe(self->lc,
                                    "MULTI_FLOOR_MAPS",
                                    multi_gridmap_handler,
                                    self);

    // --- SETUP SIDE BOX WIDGET
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_SHOW_NAVIGATOR_PLAN, 1, NULL);

    /* bot_gtk_param_widget_add_enum(self->pw, PARAM_FLOOR_NO, BOT_GTK_PARAM_WIDGET_MENU,  */
    /*                               0,  */
    /*                               "Ground",0, */
    /*                               "First",1, */
    /*                               "Second", 2,  */
    /*                               "Third",3, */
    /*                               "Fourth", 4,  */
    /*                               "Fifth", 5, */
    /*                               "Sixth", 6, */
    /*                               "Seventh", 7, */
    /*                               "Eight", 8, */
    /*                               "Ninth", 9,NULL); */

    bot_gtk_param_widget_add_enum(self->pw, PARAM_FLOOR_NO, BOT_GTK_PARAM_WIDGET_MENU,
                                  0,
                                  "Default",0, NULL);
    self->param_goal_placement = 0;
    self->param_have_goal = 0;
    self->goal[0] = 0.0;
    self->goal[1] = 0.0;
    self->param_add_loc = 0;

    check_current_floor(self);

    /*self->portal_list.list = NULL;
    self->portal_list.no_portals = 0;
    */
    self->portal_list =  NULL;
    self->goal_querry_result = NULL;
    self->topology = NULL;
    //add three buttons
    bot_gtk_param_widget_add_buttons(self->pw, QUERY_ELEVATOR, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PLACE_NAV_GOAL, NULL);
    //bot_gtk_param_widget_add_buttons(self->pw, START_ROBOT, NULL); // Doesn't do anything
    //bot_gtk_param_widget_add_buttons(self->pw, STOP_ROBOT, NULL); // Not necessary
    bot_gtk_param_widget_add_buttons(self->pw,ADD_NEW_PLACE, NULL);
    bot_gtk_param_widget_add_buttons(self->pw,ADD_PORTAL_DOOR , NULL);
    bot_gtk_param_widget_add_buttons(self->pw,ADD_PORTAL_ELEVATOR , NULL);
    //bot_gtk_param_widget_add_buttons(self->pw, QUERY_PLACE , NULL); // Nobody subscries to message


    self->param_draw_navigator_plan = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_NAVIGATOR_PLAN);
    gtk_widget_show_all(renderer->widget);
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    on_param_widget_changed(self->pw, "", self);

    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
    g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
}
