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

#include <geom_utils/geometry.h>
#include "er_renderers.h"
#include "er_gl_utils.h"
#include <bot_core/bot_core.h>
#include <carmen_utils/global.h>

#include <lcmtypes/hr_lcmtypes.h>
#include <hr_lcmtypes/lcm_channel_names.h>

#define RENDERER_NAME "Localize"

#define PARAM_REINITIALIZE "Reinitialize"
#define ADD_PERSON "Add Person"
#define PUBLISH_VALUE "Publish Value"
#define LOOK_FOR_PERSON "Look for Person"
#define PAUSE_SIMULATION "Pause Simulation"
#define RESUME_SIMULATION "Resume Simulation"
#define START_FOLLOWING "Start Following"
#define STOP_FOLLOWING "Stop Following"
#define CLEAR_PEOPLE "Clear People"
#define PERSON_INFRONT "Person in front"

#define DRAW_PERSIST_SEC 4
#define VARIANCE_THETA bot_to_radians(30.0)
//Not sure why abe has this high a variance for the angle //((2*M_PI)*(2*M_PI))

#define MIN_STD 0.3
#define MAX_STD INFINITY

typedef struct _pose_value {
    double xytheta[3]; 
    double score; 
} pose_value; 

typedef struct _RendererLocalize RendererLocalize;

struct _RendererLocalize {
    BotRenderer renderer;
    BotEventHandler ehandler;
    BotViewer *viewer;
    lcm_t *lc;

    pose_value *pose_values;
    int size_of_pose_values; 

    BotGtkParamWidget *pw;

    int dragging;
    int active; //1 = relocalize, 2 = set person location 
    point2d_t drag_start_local;
    point2d_t drag_finish_local;

    point2d_t particle_mean;
    double theta;
    double particle_std;

    GtkWidget *value_dialog;
    GtkWidget *score_entry;

    int64_t max_draw_utime;
};

static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererLocalize *self = (RendererLocalize*) renderer;
    int64_t now = bot_timestamp_now();
    if(!self->dragging && now > self->max_draw_utime && self->size_of_pose_values == 0)
        return;

    if(!(!self->dragging && now > self->max_draw_utime)){
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
    if(self->size_of_pose_values > 0){


        for(int i=0; i < self->size_of_pose_values; i++){

            glPushMatrix();
            glLineWidth(1);
            double x = self->pose_values[i].xytheta[0];
            double y = self->pose_values[i].xytheta[1];
            double theta = self->pose_values[i].xytheta[2];
            glColor3f(1, 0, 0.5);

            glTranslatef(x, y, 0);
            bot_gl_draw_circle(0.5);
            glLineWidth(2);
            glBegin(GL_LINE_STRIP);  
            glVertex2f(0.0,0.0);
            glVertex2f(0.5*cos(theta),0.5*sin(theta));
            glColor4f (0, 0, 1.0, .5);
           
            glEnd();   

            double pos[3] = {0.5, 0, 0};
            char value[128];

            glColor3f(0, 1.0, 0.0);
            sprintf(value, "%.1f", self->pose_values[i].score);

            bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_12, value,
                             BOT_GL_DRAW_TEXT_DROP_SHADOW);
            glPopMatrix();
        }
    }
}

static void
recompute_particle_distribution(RendererLocalize *self)
{
    self->particle_mean = self->drag_start_local;
    double dx = self->drag_finish_local.x - self->drag_start_local.x;
    double dy = self->drag_finish_local.y - self->drag_start_local.y;

    double theta = atan2(dy,dx);
    self->theta = theta;

    self->particle_std = sqrt(dx*dx + dy*dy);
    if(self->particle_std < MIN_STD)
        self->particle_std = MIN_STD;
    if(self->particle_std > MAX_STD)
        self->particle_std = MAX_STD;
    self->max_draw_utime = bot_timestamp_now() + DRAW_PERSIST_SEC * 1000000;
}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
             const double ray_dir[3], const GdkEventButton *event)
{
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    self->dragging = 0;

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
    }

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

void clear_people(RendererLocalize *self){
    ripl_localize_reinitialize_cmd_t msg;
    msg.utime = bot_timestamp_now();
    msg.mean[0] = 0;
    msg.mean[1] = 0;
    msg.mean[2] = 0;

    msg.variance[0] = 0;
    msg.variance[1] = 0;
    msg.variance[2] = 0;
    ripl_localize_reinitialize_cmd_t_publish(self->lc, "CLEAR_PEOPLE_CHANNEL", &msg); 
}

void send_simulator_msg(RendererLocalize *self, int state){
    //state = 0 : pause
    //state = 1 : resume 

    //we should prob send speed up/ slow down messages as well 

    ripl_simulator_cmd_t msg; 
    msg.timestamp = bot_timestamp_now();
    msg.command = state; 
    ripl_simulator_cmd_t_publish(self->lc, "SIMULATOR_CMD", &msg);
}

static
void add_value_button(GtkWidget *button __attribute__ ((unused)),
                      gpointer user_data __attribute__ ((unused)))
{
    RendererLocalize *self = (RendererLocalize*) user_data;
    char *errs;
    char name[100];

    double score = gtk_range_get_value((GTK_RANGE(self->score_entry)));
    
    fprintf(stderr, "Pose : %f,%f,%f => %f\n", self->particle_mean.x, self->particle_mean.y, self->theta, score);
       
    ripl_pose_value_t msg; 
    msg.utime = bot_timestamp_now(); 
    msg.x = self->particle_mean.x;
    msg.y = self->particle_mean.y;
    msg.theta = self->theta;
    msg.score = score; 
    
    //this should be added to a list and displayed 

    ripl_pose_value_t_publish(self->lc, "POSE_VALUE", &msg);

    self->pose_values = (pose_value *) realloc(self->pose_values, (self->size_of_pose_values +1) * sizeof(pose_value));

    self->pose_values[self->size_of_pose_values].xytheta[0] = self->particle_mean.x;
    self->pose_values[self->size_of_pose_values].xytheta[1] = self->particle_mean.y;
    self->pose_values[self->size_of_pose_values].xytheta[2] = self->theta;
    self->pose_values[self->size_of_pose_values].score = score;

    self->size_of_pose_values++;

    gtk_widget_destroy(self->score_entry);
    gtk_widget_destroy(self->value_dialog);
   
}


void start_ask_value(RendererLocalize *self)
{  
    fprintf(stderr, "Creating Dialog\n");
    //just ask for a score 
    self->score_entry = gtk_hscale_new_with_range(-10, 10, 0.5); 

    GtkWidget *vbox, *hbox, *name_label, *button;
    //remember to destroy
    self->value_dialog = gtk_dialog_new();
    vbox = gtk_vbox_new(FALSE, 0);
    hbox = gtk_hbox_new(FALSE, 0);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (self->value_dialog)->vbox),
                        vbox, TRUE, TRUE, 0);
    name_label = gtk_label_new("Enter Score for Pose: ");
    gtk_box_pack_start (GTK_BOX (vbox), name_label, TRUE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX(vbox),  self->score_entry, TRUE, TRUE, 0);
  
    button = gtk_button_new_with_label("OK");
    gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);
  
    gtk_signal_connect(GTK_OBJECT(button), "clicked", 
                       (GtkSignalFunc)add_value_button, self);
  
    button = gtk_button_new_with_label("Cancel");
    gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);	

    gtk_box_pack_start (GTK_BOX(vbox),  hbox, TRUE, TRUE, 0);
  
    gtk_signal_connect_object(GTK_OBJECT(button),
                              "clicked", (GtkSignalFunc)gtk_widget_destroy, 
                              (gpointer)self->value_dialog);	
    self->active = 0;

    gtk_widget_show_all(self->value_dialog);
}

static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventButton *event)
{
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    if (self->dragging) {
        self->dragging = 0;
    }
    if (self->active == 3){
        //ask for value 
        start_ask_value(self);
    }
    
    if (self->active != 0 && self->active < 3) {
        // check drag points and publish
        ripl_localize_reinitialize_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.mean[0] = self->particle_mean.x;
        msg.mean[1] = self->particle_mean.y;
        msg.mean[2] = self->theta;
      
        double v = self->particle_std * self->particle_std;
        msg.variance[0] = v;
        msg.variance[1] = v;
        msg.variance[2] = VARIANCE_THETA;
      
        fprintf(stderr,"Localizer Button Released => Activate Value : %d\n", self->active);
        if(self->active == 1){
            fprintf(stderr, "Reinitializing \n");
            ripl_localize_reinitialize_cmd_t_publish(self->lc, LOCALIZE_REINITIALIZE_CHANNEL, &msg);
        }
        else if (self->active == 2){
            //clear the current location 
            clear_people(self);
            //send new location 
            ripl_localize_reinitialize_cmd_t_publish(self->lc, "CREATE_PEOPLE_CHANNEL", &msg);
        }

        bot_viewer_set_status_bar_message(self->viewer, "");
        self->active = 0;

        ehandler->picking = 0;
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
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

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

void activate(RendererLocalize *self, int type)
{
    self->active = type;
    if(type==1){
        bot_viewer_set_status_bar_message(self->viewer, 
                                          "Click and drag to initialize new particles");
    }
    if(type==2){
        bot_viewer_set_status_bar_message(self->viewer, 
                                          "Click and drag to initialize new person");
    }
    if(type==3){
        bot_viewer_set_status_bar_message(self->viewer, 
                                          "Click and drag to initialize to add new training example");
    }
    
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
                      const GdkEventKey *event)
{
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    if ((event->keyval == 'l' || event->keyval == 'L') && self->active==0) {
        activate(self,1);
        bot_viewer_request_pick (viewer, ehandler);
    } else if(event->keyval == GDK_Escape) {
        self->active = 0;
        ehandler->picking = 0;
        bot_viewer_set_status_bar_message(self->viewer, "");
    }

    return 0;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererLocalize *self = (RendererLocalize*) user;

    if(!strcmp(name, PARAM_REINITIALIZE)) {
        fprintf(stderr,"Clicked activate - Reinitialize robot\n");
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        activate(self, 1);
    }
    if(!strcmp(name, ADD_PERSON)) {
        fprintf(stderr,"Clicked Activate - Set person Location\n");
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        activate(self, 2);
    }
    if(!strcmp(name, PUBLISH_VALUE)) {
        fprintf(stderr,"Clicked Activate - Add training example\n");
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        activate(self, 3);
    }

    if(!strcmp(name, CLEAR_PEOPLE)) {
        fprintf(stderr,"Clearing People\n");
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        clear_people(self);
    }

    if(!strcmp(name, PAUSE_SIMULATION)) {
        fprintf(stderr,"Pausing Simulation\n");
        send_simulator_msg(self, 0);
    }

    if(!strcmp(name, RESUME_SIMULATION)) {
        fprintf(stderr,"Resuming Simulation\n");
        send_simulator_msg(self, 1);
    }

    if(!strcmp(name, START_FOLLOWING)) {
        fprintf(stderr,"Start Following\n");
        ripl_speech_cmd_t msg;
        msg.cmd_type = "FOLLOWER";
        msg.cmd_property = "START_FOLLOWING";
        ripl_speech_cmd_t_publish(self->lc, "PERSON_TRACKER", &msg);        
    }  

     if(!strcmp(name, STOP_FOLLOWING)) {
        fprintf(stderr,"Stop Following\n");
        ripl_speech_cmd_t msg;
        msg.cmd_type = "FOLLOWER";
        msg.cmd_property = "IDLE";
        ripl_speech_cmd_t_publish(self->lc, "PERSON_TRACKER", &msg);        
    }  

    if(!strcmp(name, LOOK_FOR_PERSON)) {
        fprintf(stderr,"Clicked Activate - Look for person\n");
        ripl_speech_cmd_t msg;
        msg.cmd_type = "TRACKER";
        msg.cmd_property = "ASKED_TO_FRONT";
        ripl_speech_cmd_t_publish(self->lc, "PERSON_TRACKER", &msg);        
    }  
    if(!strcmp(name, PERSON_INFRONT)) {
        fprintf(stderr,"Clicked Activate - Person is in front\n");
        ripl_person_tracking_cmd_t msg; 
        msg.utime = bot_timestamp_now();
        msg.command = ERLCM_PERSON_TRACKING_CMD_T_CMD_PERSON_IN_FRONT; 
        msg.sender = ERLCM_PERSON_TRACKING_CMD_T_SENDER_DM; 
        ripl_person_tracking_cmd_t_publish(self->lc, "PERSON_TRACKING_CMD", &msg);        
    }  
}

static void
_free (BotRenderer *renderer)
{
    free (renderer);
}

BotRenderer *renderer_localize_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererLocalize *self = (RendererLocalize*) calloc (1, sizeof (RendererLocalize));
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

    self->lc = lcm; //globals_get_lcm_full(NULL,1);

    
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_REINITIALIZE, NULL);

    //add clear people and add people button 
    bot_gtk_param_widget_add_separator(self->pw, "Person Tracker");

    bot_gtk_param_widget_add_buttons(self->pw, LOOK_FOR_PERSON, NULL);
    
    bot_gtk_param_widget_add_buttons(self->pw, PERSON_INFRONT, NULL);

    bot_gtk_param_widget_add_separator(self->pw, "Person Follower");

    bot_gtk_param_widget_add_buttons(self->pw, START_FOLLOWING, NULL);

    bot_gtk_param_widget_add_buttons(self->pw, STOP_FOLLOWING, NULL);

    bot_gtk_param_widget_add_separator(self->pw, "Simulator");
    
    bot_gtk_param_widget_add_buttons(self->pw, ADD_PERSON, NULL);

    bot_gtk_param_widget_add_buttons(self->pw, PUBLISH_VALUE, NULL);

    bot_gtk_param_widget_add_buttons(self->pw, CLEAR_PEOPLE, NULL);

    //pause/resume simulation buttons 
    bot_gtk_param_widget_add_buttons(self->pw, PAUSE_SIMULATION, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, RESUME_SIMULATION, NULL);
  

    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->renderer.widget = GTK_WIDGET(self->pw);

    self->pose_values = NULL;
    self->size_of_pose_values = 0;
    self->active = 0;

    return &self->renderer;
}

void localize_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    bot_viewer_add_renderer(viewer, renderer_localize_new(viewer, render_priority, lcm), 
                            render_priority);
}
