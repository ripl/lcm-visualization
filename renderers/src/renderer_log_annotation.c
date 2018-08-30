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
#include <bot_frames/bot_frames.h>
#include <carmen_utils/global.h>

#include <lcmtypes/hr_lcmtypes.h>
#include <hr_lcmtypes/lcm_channel_names.h>
#include <lcmtypes/ripl_servo_position_list_t.h>
#include <lcmtypes/ripl_annotation_t.h> 
#include <lcmtypes/ripl_log_annotate_msg_t.h>
#include <lcmtypes/bot_core_pose_t.h>

#define DIST_TO_ADD 0.05
#define RENDERER_NAME "LOG_ANNOTATION"

#define LANDMARKS "Landmarks"
#define ANNOTATION_START "Annotation_Start"
#define ANNOTATION_PAUSE "Annotation_End"

#define SET_POSE_QUEUE "Set Pose Queue"
#define MAX_POSE_QUEUE_SIZE 2000

typedef struct _RendererLogAnnotation RendererLogAnnotation;

struct _RendererLogAnnotation {
    BotRenderer renderer;
    BotEventHandler ehandler;
    BotViewer *viewer;
    lcm_t *lc;
    BotFrames  *atrans;
    BotParam   *config;

    BotGtkParamWidget *pw;
    int numLandmarks;
    char ** landmarkNames;
    int * landmarkNums;
    int activeLandmarkNum;

    int start_pressed;               //If start or end pressed, redraw.
    int end_pressed;

    double start_position_body[3];      //start position
    double start_position_local[3];

    double end_position_body[3];       //end position
    double end_position_local[3];

    int annotation_started;  //0 not started, 1 started. reset to 0 when annotation ends
    int64_t annotation_start_utime;

    //    ripl_annotation_t_subscription_t *sub;
    bot_core_pose_t_subscription_t *pose_sub;

    //    ripl_annotation_t *annotation;
    bot_core_pose_t *pose_last;

    bot_core_pose_t **annotated_pose_list;
    int no_annotated_pose_list;

    char *last_annotation_str; 
    int pose_history_length;
    BotPtrCircular *pose_history;
};

static void pose_destroy(void *user, void *p)
{
  if (p) {
    bot_core_pose_t *pose = (bot_core_pose_t*) p;
    bot_core_pose_t_destroy(pose);
  }
}

////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Drawing Functions ------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void _draw(BotViewer *viewer, BotRenderer *renderer){

    RendererLogAnnotation *self = (RendererLogAnnotation*)renderer;
		
    //glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    glColor3f(1, 0, 0);
    
    if(self->start_pressed || self->end_pressed){
        glPushMatrix();
        glTranslatef(self->start_position_body[0], self->start_position_body[1], 0);
        bot_gl_draw_circle(0.1);
        glPopMatrix();

        if(!self->start_pressed){
            glColor3f(0, 0, 1);
            glPushMatrix();
            //		fprintf(stderr, "Prepare to draw end_position:%.2f %.2f\n", self->end_position_local[0], self->end_position_local[1]);
            glTranslatef(self->end_position_body[0], self->end_position_body[1], 0);
            bot_gl_draw_circle(0.1);
            glPopMatrix();
            
        }
        if(self->last_annotation_str != NULL){
            double textpos[3] = {self->start_position_body[0] + 0.1, self->start_position_body[1]+ 0.1, 0};
            bot_gl_draw_text(textpos, GLUT_BITMAP_HELVETICA_12, self->last_annotation_str,
                             BOT_GL_DRAW_TEXT_DROP_SHADOW);
        }
    }
    

    /*if(self->end_pressed){
        glColor3f(0, 1, 0);
        glBegin(GL_LINES);
        glVertex2d(self->start_position_body[0], self->start_position_body[1]);
        glVertex2d(self->end_position_body[0], self->end_position_body[1]);
        glEnd();
        }*/

    if(self->no_annotated_pose_list > 0 && self->annotated_pose_list != NULL){
        //draw points 
        glColor3f(0, 1, 0);
        //glLineWidth(10);
        glPointSize(8);
        glBegin (GL_POINTS);
        for(int i=0; i < self->no_annotated_pose_list ; i++){
            glVertex3f (self->annotated_pose_list[i]->pos[0],
                        self->annotated_pose_list[i]->pos[1],
                        self->annotated_pose_list[i]->pos[2]);
        }
        glEnd();
        glLineWidth(5);
        glBegin (GL_LINE_STRIP);
        for(int i=0; i < self->no_annotated_pose_list ; i++){
            glVertex3f (self->annotated_pose_list[i]->pos[0],
                        self->annotated_pose_list[i]->pos[1],
                        self->annotated_pose_list[i]->pos[2]);
        }
        glEnd();
    }
}

void reset_pose_list(RendererLogAnnotation *self){
    if(self->annotated_pose_list != NULL && self->no_annotated_pose_list > 0){
        for(int i=0; i < self->no_annotated_pose_list ; i++){
            bot_core_pose_t_destroy(self->annotated_pose_list[i]);
        }
        self->no_annotated_pose_list = 0;
        free(self->annotated_pose_list);
        self->annotated_pose_list = NULL;
    }
}


static void 
on_pose(const lcm_recv_buf_t *rbuf, 
	const char *channel, const bot_core_pose_t *msg, void *user)
{
    RendererLogAnnotation *self = (RendererLogAnnotation*)user;

    if (self->pose_last)
        bot_core_pose_t_destroy (self->pose_last);

    self->pose_last = bot_core_pose_t_copy(msg);

    //if the pose has changed significantly - we should add it to queue 
    if(bot_ptr_circular_size(self->pose_history)){
        bot_core_pose_t *l_added_pose = bot_ptr_circular_index(self->pose_history, 0);
        
        //add to the queue if we have moved enough 
        if(hypot(l_added_pose->pos[0] - self->pose_last->pos[0], l_added_pose->pos[1] - self->pose_last->pos[1]) > DIST_TO_ADD){
            bot_ptr_circular_add(self->pose_history, bot_core_pose_t_copy(self->pose_last));
        }
    }
    else{
        bot_ptr_circular_add(self->pose_history, bot_core_pose_t_copy(self->pose_last));
    }
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererLogAnnotation *self = (RendererLogAnnotation*) user;

    if(self->pose_history_length != bot_gtk_param_widget_get_int(self->pw, SET_POSE_QUEUE)){
        self->pose_history_length = bot_gtk_param_widget_get_int(self->pw, SET_POSE_QUEUE);
        //trash the old queue and create a new one 
        bot_ptr_circular_resize(self->pose_history, self->pose_history_length);
    }

    if(!strcmp(name, LANDMARKS)) {
        self->activeLandmarkNum = bot_gtk_param_widget_get_enum(pw, LANDMARKS);
        bot_viewer_set_status_bar_message(self->viewer, "Landmark:%d", self->activeLandmarkNum);
    }
    if(!strcmp(name, ANNOTATION_START)){
        if(!self->pose_last)
            return;

        if( self->pose_last->utime < self->annotation_start_utime){
            fprintf(stderr, "New start called - ignoring the old start time\n");
            self->annotation_started = 0;
        }

        //ensure previous annotation has ended 
        if(!self->annotation_started){
            self->annotation_started = 1;

            //clear the pose history
            bot_ptr_circular_clear(self->pose_history);

            reset_pose_list(self);

            self->start_pressed = 1;
            self->end_pressed = 0;
            //Initialize a message
            self->annotation_start_utime = self->pose_last->utime;
	
            //Get current position in BODY frame
            for(int i = 0; i < 3; i++){
                self->start_position_body[i] = self->pose_last->pos[i];
            }

            if(self->last_annotation_str != NULL)
                free(self->last_annotation_str);

            self->last_annotation_str = strdup(self->landmarkNames[self->activeLandmarkNum]);

            fprintf(stderr,"Clicked start - Start Annotation\n");

            //Transform from BODY frame to LOCAL frame
            int trans = bot_frames_transform_vec(self->atrans, "body", "local", self->start_position_body, self->start_position_local);
		
            if(trans == 1){
                fprintf(stderr,"Start Position Transformation Successfully.\n");       //If == 1, transform successfully
                fprintf(stderr,"Start_Position_Body_Frame:%.2f %.2f %.2f; Start_Position_Local_Frame:%.2f %.2f %.2f\n", self->start_position_body[0], self->start_position_body[1], self->start_position_body[2], self->start_position_local[0], self->start_position_local[1], self->start_position_local[2]);		
            }
            else{
                fprintf(stderr,"Start Position Transformation Failed.\n"); 
            }


            //Mark the start position
            if(self->start_pressed){
                //			fprintf(stderr, "In the if statement.\n");
                bot_viewer_request_redraw(self->viewer);
            }

        }
        else{
            fprintf(stderr, "Already started annotation.\n");
        }
    }
    else if(!strcmp(name, ANNOTATION_PAUSE)){
        if(self->pose_last->utime < self->annotation_start_utime){
            fprintf(stderr, "Annotation last utime is earlier than the start time - log rewinded? - trashing the label\n");
            self->annotation_started = 0;  
            self->start_pressed = 0;
            self->end_pressed = 0;
        } 

        //ensure annotation has started before end
        if (self->annotation_started){
            self->annotation_started = 0;
            self->start_pressed = 0;
            self->end_pressed = 1;
            ripl_annotation_t msg;
            msg.start_utime = self->annotation_start_utime;
            msg.end_utime = self->pose_last->utime;
            msg.utime = bot_timestamp_now();

            char annotation[100];
            strcpy(annotation, self->landmarkNames[self->activeLandmarkNum]);

            

            msg.annotation = strdup(annotation);
                
            char *channel_name = "POSE";
            msg.reference_channel = strdup (channel_name);

            for(int i = 0; i < 3; i++){
                self->end_position_body[i] = self->pose_last->pos[i];
            }

            //copy the pose list to another data structure (which should be cleared when a new annotation is happening)

		
            fprintf(stderr,"Clicked end - End Annotation\n");

            //Transform from BODY to LOCAL
            int trans = bot_frames_transform_vec(self->atrans, "body", "local", self->end_position_body, self->end_position_local);
            if(trans == 1){
                fprintf(stderr,"End Position Transformation Successfully.\n");       //If == 1, transform successfully
                fprintf(stderr,"End_Position_Body_Frame:%.2f %.2f %.2f; End_Position_Local_Frame:%.2f %.2f %.2f\n", self->end_position_body[0], self->end_position_body[1], self->end_position_body[2], self->end_position_local[0], self->end_position_local[1], self->end_position_local[2]);		
            }
            else{
                fprintf(stderr,"End Position Transformation Failed.\n"); 
            }
		
            


            //Publish the LOG_ANNOTAION message
            ripl_annotation_t_publish(self->lc, "LOG_ANNOTATION", &msg);

            /*ripl_annotation_msg_t a_msg;
              a_msg.utime = msg.utime;
              a_msg.start_time = msg.start_utime;
              a_msg.end_utime = msg.end_utime;
              a_msg.type = */

            free (msg.annotation);
            free (msg.reference_channel);

            int matched_ind = 0;
            int count = 0;
            for(int i=0; i < bot_ptr_circular_size(self->pose_history); i++){
                bot_core_pose_t *pose = bot_ptr_circular_index(self->pose_history, i);
                if(pose->utime < msg.end_utime && pose->utime > msg.start_utime){
                    count++;
                }      
                if(pose->utime < msg.start_utime)
                    break;
            }

            self->annotated_pose_list = (bot_core_pose_t **) calloc(count, sizeof(bot_core_pose_t *));
            
            for(int i=0; i < bot_ptr_circular_size(self->pose_history); i++){
                bot_core_pose_t *pose = bot_ptr_circular_index(self->pose_history, i);
                if(pose->utime < msg.end_utime && pose->utime > msg.start_utime){
                    self->annotated_pose_list[i] = bot_core_pose_t_copy(pose);
                    self->no_annotated_pose_list = i+1;
                }   
                if(pose->utime < msg.start_utime)
                    break;
            }

            if(self->end_pressed){
                bot_viewer_request_redraw(self->viewer);
            }
        }
        else{
            fprintf(stderr, "Haven't started annotation\n");
        }
    }
}



static void
_free (BotRenderer *renderer)
{

    RendererLogAnnotation *self = (RendererLogAnnotation *) renderer->user;
    reset_pose_list(self);
    if(self->pose_history)
        bot_ptr_circular_destroy(self->pose_history);
    free(self);
    free (renderer);
}

BotRenderer *renderer_log_annotation_new (BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param)
{
    RendererLogAnnotation *self = (RendererLogAnnotation*) calloc (1, sizeof (RendererLogAnnotation));
    self->viewer = viewer;
    self->renderer.draw = _draw;
    self->renderer.destroy = _free;
    self->renderer.name = RENDERER_NAME;
    self->renderer.user = self;

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->key_press = NULL;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = NULL;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    self->lc = lcm; //globals_get_lcm_full(NULL,1);

    //Try to set up frame
    self->config = param;
    self->atrans = bot_frames_get_global(self->lc, self->config);//globals_get_atrans();


    //subscribe to pose_t lcm messages
    self->pose_sub = bot_core_pose_t_subscribe(self->lc, "POSE", on_pose, self);

    
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    self->no_annotated_pose_list = 0;
    self->annotated_pose_list = NULL;

    //Change landmarks here
    self->numLandmarks = 9;  //one more for null
    self->landmarkNames = calloc(self->numLandmarks + 1, sizeof(char*));
    self->landmarkNames[1] = "doorway";
    self->landmarkNames[2] = "conference_room";
    self->landmarkNames[3] = "office";
    self->landmarkNames[4] = "lab";
    self->landmarkNames[5] = "open_area";
    self->landmarkNames[6] = "hallway";
    self->landmarkNames[7] = "corridor";
    self->landmarkNames[8] = "large_meeting_room";
    self->landmarkNums = calloc(self->numLandmarks, sizeof(int));

    self->last_annotation_str = NULL;
    for (int i = 0; i < self->numLandmarks; i++){
        self->landmarkNums[i] = i;
    }

    //start/pause log annotation
    bot_gtk_param_widget_add_enumv(self->pw, LANDMARKS, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, self->numLandmarks, (const char **) self->landmarkNames, self->landmarkNums);
    bot_gtk_param_widget_add_buttons(self->pw, ANNOTATION_START, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, ANNOTATION_PAUSE, NULL);

    bot_gtk_param_widget_add_int(self->pw, SET_POSE_QUEUE, BOT_GTK_PARAM_WIDGET_SLIDER, 0, MAX_POSE_QUEUE_SIZE, 1, MAX_POSE_QUEUE_SIZE);
    
    self->pose_history_length = bot_gtk_param_widget_get_int(self->pw, SET_POSE_QUEUE);
    self->pose_history = bot_ptr_circular_new(self->pose_history_length, pose_destroy, NULL);

    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->renderer.widget = GTK_WIDGET(self->pw);

    self->activeLandmarkNum = 0;

    return &self->renderer;
}

void log_annotation_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param)
{
    bot_viewer_add_renderer(viewer, renderer_log_annotation_new(viewer, render_priority, lcm, param), 
                            render_priority);
}
