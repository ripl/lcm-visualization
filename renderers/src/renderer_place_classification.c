/*
 * Renders SVM without HMM, SVM with HMM classification results when receive place_classification_t or HMM_place_classification_t messages
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
#include <bot_vis/gtk_util.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gl_util.h>
#include <bot_vis/scrollplot2d.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/erlcm_place_classification_t.h>

#define BIN_SIZE 1000
#define NO_CLASS 8
#define MAX_NO_CLASS 10.0
#define DEFAULT_DIST 0.4

#define DISTANCE_TO_DRAW "Node Distance" 
#define RAW_CLASSIFICATION "Raw Classification"
#define HMM_CLASSIFICATION "HMM Classification"
#define DOORWAY_DETECTIONS "Draw Doorways"
#define DRAW_MAX_LIKELIHOOD "Draw Max Likelihood"
#define DRAW_PIE_CHART "Draw Pie Chart"
#define DRAW_TEXT "Draw Text"
#define DRAW_HISTORY_SIZE "Place History Size"
#define CLEAR_HISTORY "Clear History"
#define COLOR_LINEAR_SCALE "Scale Colors" 

typedef struct _RendererPlaceClassification RendererPlaceClassification;

typedef struct _classification_t{
    erlcm_place_classification_t *class;
    double pos[2];
} classification_t;

struct _RendererPlaceClassification {
    lcm_t *lcm;
    BotRenderer renderer;
    BotViewer *viewer;
    BotFrames *atrans;
    BotParam *config;
    BotGtkParamWidget *pw;

    int no_label;
    double dist;
    int scale_colors;

    BotPtrCircular *classifications;
    BotPtrCircular *HMM_classifications;

    erlcm_place_classification_t *last_svm;
    erlcm_place_classification_t *last_HMM;

    erlcm_place_classification_t_subscription_t *label_sub;
    erlcm_place_classification_t_subscription_t *HMM_sub;
    double transition_matrix[16];
    double pos[3];
};


void destroy_classification(classification_t *np ){
    erlcm_place_classification_t_destroy(np->class);
    free(np);
}

void
circ_free_classification_data(void *user, void *p){
    classification_t *np = (classification_t *)p;
    destroy_classification(np);
}

static void 
on_classification(const lcm_recv_buf_t *rbuf, 
		  const char *channel, const erlcm_place_classification_t *msg, void *user)
{
    RendererPlaceClassification *self = (RendererPlaceClassification *)user;
  
    if(self->last_svm != NULL){
        /*if(msg->sensor_utime - self->last_svm->sensor_utime > pow(10, 6) || self->last_svm->sensor_utime - msg->sensor_utime > pow(10, 6)){
          fprintf(stderr, "Difference in utime: %lld\n", msg->sensor_utime - self->last_svm->sensor_utime);
          bot_ptr_circular_clear(self->classifications);
          fprintf(stderr, "Clear original classification buffer.\n");
          }*/
        erlcm_place_classification_t_destroy(self->last_svm);
    }
   
    self->last_svm = erlcm_place_classification_t_copy(msg);
    classification_t *cl = (classification_t *) calloc(1, sizeof(classification_t));

    cl->class = erlcm_place_classification_t_copy(msg);

    BotTrans body_to_local;
    bot_frames_get_trans_with_utime(self->atrans, "body", "local", msg->sensor_utime, &body_to_local);
    cl->pos[0] = body_to_local.trans_vec[0];
    cl->pos[1] = body_to_local.trans_vec[1];

    //we can decimate here - its better than doing it in the draw 
    if(bot_ptr_circular_size(self->classifications) > 0){
        classification_t *cl_last = (classification_t *) bot_ptr_circular_index(self->classifications, 0);
        
        if(hypot(cl_last->pos[0] - cl->pos[0], cl_last->pos[1] - cl->pos[1]) > self->dist){
            bot_ptr_circular_add(self->classifications, cl);
        }
        else{
            destroy_classification(cl);
        }
    }
    else{
        bot_ptr_circular_add(self->classifications, cl);
    }

    if(bot_gtk_param_widget_get_bool(self->pw, RAW_CLASSIFICATION))
        bot_viewer_request_redraw(self->viewer);
}

static void
on_HMM(const lcm_recv_buf_t *rbuf, 
       const char *channel, const erlcm_place_classification_t *msg, void *user)
{
    RendererPlaceClassification *self = (RendererPlaceClassification *)user;

  
    if(self->last_HMM != NULL){
        /*
          if(msg->sensor_utime - self->last_HMM->sensor_utime > pow(10, 6) || self->last_HMM->sensor_utime - msg->sensor_utime > pow(10, 6)){
          fprintf(stderr, "Difference in utime: %lld\n", msg->sensor_utime - self->last_HMM->sensor_utime);
          bot_ptr_circular_clear(self->HMM_classifications);
          fprintf(stderr, "Clean original HMM_classification buffer.\n");
          }
        */
        erlcm_place_classification_t_destroy(self->last_HMM);
    }
   
    self->last_HMM = erlcm_place_classification_t_copy(msg);
  
    classification_t *cl = (classification_t *) calloc(1, sizeof(classification_t));

    cl->class = erlcm_place_classification_t_copy(msg);

    BotTrans body_to_local;
    bot_frames_get_trans_with_utime(self->atrans, "body", "local", msg->sensor_utime, &body_to_local);
    cl->pos[0] = body_to_local.trans_vec[0];
    cl->pos[1] = body_to_local.trans_vec[1];

    //fprintf(stderr, "Result at : %f,%f\n", cl->pos[0], cl->pos[1]);
    //we can decimate here - its better than doing it in the draw 
    if(bot_ptr_circular_size(self->HMM_classifications) > 0){
        classification_t *cl_last = (classification_t *) bot_ptr_circular_index(self->HMM_classifications, 0);
        
        if(hypot(cl_last->pos[0] - cl->pos[0], cl_last->pos[1] - cl->pos[1]) > self->dist){
            bot_ptr_circular_add(self->HMM_classifications, cl);
        }
        else{
            destroy_classification(cl);
        }
    }
    else{
        bot_ptr_circular_add(self->HMM_classifications, cl);
    }

    if(bot_gtk_param_widget_get_bool(self->pw, HMM_CLASSIFICATION))
        bot_viewer_request_redraw(self->viewer);
}

//no_class is 8 in this drawing routine. If no of class is different, 
// might need to consider use more colors
static void 
draw_pie_chart(double radius, erlcm_place_classification_t *cl, RendererPlaceClassification *self){//double *values, BotRenderer *rd, int no_class)
    double centerx = 0;
    double centery = 0;
    

    //Doorway drawing is tick & has doorway class.
    if(bot_gtk_param_widget_get_bool(self->pw, DOORWAY_DETECTIONS)){

        glBegin(GL_POLYGON);
        glColor3f(1,0,0);
        glVertex2f(centerx, centery);
        for(double j = 0; j < 2 * M_PI; j += 0.1){
            double x = radius * sin(j);
            double y = radius * cos(j);
            glVertex2f(x, y);
        }
        glEnd();
        return;
    }

    if(!bot_gtk_param_widget_get_bool(self->pw, DRAW_MAX_LIKELIHOOD)){
        glBegin(GL_POLYGON);
        double theta = 0;
        for(int i = 0;i < cl->no_class;i++){
            erlcm_place_classification_class_t *curr_cl = &cl->classes[i];
            double val;
            if(!self->scale_colors)
                val = fmin(1, curr_cl->name/ MAX_NO_CLASS);
            else
                val = fmin(1, i/(double) cl->no_class);
            float *color = bot_color_util_jet(val);
            glColor3f(color[0], color[1], color[2]);//r, g, b);
            double value = curr_cl->probability;
            glVertex2f(centerx, centery);
            for(double j = theta; j < theta + (double)2 * M_PI * value; j += 0.1){
                double x = radius * sin(j);
                double y = radius * cos(j);
                glVertex2f(x, y);
            }
            theta += (double)2 * M_PI * value;
        }
        glEnd();
    }
    else{
        double max = -1;
        int index = -1;
        int class_id = -1;
        for(int i = 0; i < cl->no_class; i++){
            erlcm_place_classification_class_t *curr_cl = &cl->classes[i];
            double value = curr_cl->probability;
            if(value > max){
                max = value;
                class_id = curr_cl->name;
                index = i;
            }
        }
        double val;
        if(!self->scale_colors)
            val = fmin(1, class_id/ MAX_NO_CLASS);
        else
            val = fmin(1, index/(double) cl->no_class);
        
        float *color = bot_color_util_jet(val);
        glColor3f(color[0], color[1], color[2]);
        
        bot_gl_draw_disk(radius);
    }
}


static void 
    _draw(BotViewer *viewer, BotRenderer *r)
{
    RendererPlaceClassification *self = (RendererPlaceClassification *)r->user;
  
    void *font = GLUT_BITMAP_8_BY_13;
    glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    //if circular buffer does not have enough labels to display, return.
    /*
      if(bot_ptr_circular_size(self->classifications) < self->no_label)
      return;

      /*
      //Not a good idea to return. This will terminate drawing of result without HMM. 
      if(bot_ptr_circular_size(self->HMM_classifications) < self->no_label)
      return;
    */

    int valid_classification = 0;
    int valid_HMM = 0;

    if(bot_ptr_circular_size(self->classifications) > 0 && bot_gtk_param_widget_get_bool(self->pw, RAW_CLASSIFICATION)){
       
        //Check through the classification circular buffer
        for(int i = 0; i < (bot_ptr_circular_size(self->classifications));i++){
            //if have drawn no_label classification labels, break
            //if(valid_classification == self->no_label)
            //break;
    
            classification_t *cl = (classification_t *) bot_ptr_circular_index(self->classifications, i);
            
            erlcm_place_classification_t *l = cl->class;
            
            
            char label[256];
      
            /*double prob[l->no_class];
            for(int j = 0; j < l->no_class; j++)
            prob[j] = l->classes[j].probability;*/
      
                
            //sprintf(label, "Prediction:%s\nelevator:%.2f\nconference_room:%.2f\noffice:%.2f\nlab:%.2f\nopen_area:%.2f\nhallway:%.2f\ncorridor:%.2f\nlarge_meeting_room:%.2f", l->label, prob[0], prob[1], prob[2], prob[3], prob[4], prob[5], prob[6], prob[7]);
      
            //If text box is tick, draw text
            if(bot_gtk_param_widget_get_bool(self->pw, DRAW_TEXT)){
                /*glColor3f(0,1,0); 
                  bot_gl_draw_text(self->pos, NULL, label,
                  //                     GLUTIL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES |
                  BOT_GL_DRAW_TEXT_JUSTIFY_CENTER |
                  BOT_GL_DRAW_TEXT_ANCHOR_VCENTER |
                  BOT_GL_DRAW_TEXT_ANCHOR_HCENTER |
                  BOT_GL_DRAW_TEXT_DROP_SHADOW);*/
            }

            //If pie chart box is tick, draw pie charts 
            if(bot_gtk_param_widget_get_bool(self->pw, DRAW_PIE_CHART)){
                glPushMatrix();
                glTranslatef(cl->pos[0], cl->pos[1], 0);
                draw_pie_chart(0.2, l, self);//prob, r, l->no_class);
                glPopMatrix();
            }
                
            valid_classification++;
        }
    }

    if(bot_ptr_circular_size(self->HMM_classifications) > 0 && bot_gtk_param_widget_get_bool(self->pw, HMM_CLASSIFICATION)){
        
        valid_classification = 0;
        //Check through the classification circular buffer
        for(int i = 0; i < (bot_ptr_circular_size(self->HMM_classifications));i++){
            //if have drawn no_label classification labels, break
            //if(valid_classification == self->no_label)
            //break;
    
            classification_t *cl = (classification_t *) bot_ptr_circular_index(self->HMM_classifications, i);
            
            erlcm_place_classification_t *l = cl->class;
            
            char label[256];
            
            //double prob[l->no_class];
            //for(int j = 0; j < l->no_class; j++)
            // prob[j] = l->classes[j].probability;
            
                
            //sprintf(label, "Prediction:%s\nelevator:%.2f\nconference_room:%.2f\noffice:%.2f\nlab:%.2f\nopen_area:%.2f\nhallway:%.2f\ncorridor:%.2f\nlarge_meeting_room:%.2f", l->label, prob[0], prob[1], prob[2], prob[3], prob[4], prob[5], prob[6], prob[7]);
      
            //If text box is tick, draw text
            if(bot_gtk_param_widget_get_bool(self->pw, DRAW_TEXT)){
                /*glColor3f(0,1,0); 
                  bot_gl_draw_text(self->pos, NULL, label,
                  //                     GLUTIL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES |
                  BOT_GL_DRAW_TEXT_JUSTIFY_CENTER |
                  BOT_GL_DRAW_TEXT_ANCHOR_VCENTER |
                  BOT_GL_DRAW_TEXT_ANCHOR_HCENTER |
                  BOT_GL_DRAW_TEXT_DROP_SHADOW);*/
            }

            //If pie chart box is tick, draw pie charts 
            if(bot_gtk_param_widget_get_bool(self->pw, DRAW_PIE_CHART)){
                glPushMatrix();
                glTranslatef(cl->pos[0], cl->pos[1], 0.2);
                draw_pie_chart(0.2, l, self);
                //draw_pie_chart(0.2, prob, r, l->no_class);
                glPopMatrix();
            }
                
            valid_classification++;
        }
    }
    glPopAttrib();
}


static void 
_destroy(BotRenderer *r)
{
    if(!r) return;
  
    RendererPlaceClassification *self = (RendererPlaceClassification *)r->user;
    free(self);
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererPlaceClassification *self = (RendererPlaceClassification *)user;
  
    if(!strcmp(name, "NO_LABEL")){
        self->no_label = (int)bot_gtk_param_widget_get_double(pw, "NO_LABEL");
        fprintf(stderr, "No of labels displayed: %d\n", self->no_label);
    }
    if(!strcmp(name, DISTANCE_TO_DRAW)){
        self->dist = bot_gtk_param_widget_get_double(pw, DISTANCE_TO_DRAW);
    }
    if(!strcmp(name, COLOR_LINEAR_SCALE)){
        self->scale_colors = bot_gtk_param_widget_get_bool(pw, COLOR_LINEAR_SCALE);
    }

    if(!strcmp(name, CLEAR_HISTORY)){
        bot_ptr_circular_clear(self->classifications);
        bot_ptr_circular_clear(self->HMM_classifications);
    }

    bot_viewer_request_redraw(self->viewer);
}



BotRenderer *renderer_place_classification_new (BotViewer *viewer, BotParam *param)
{
    RendererPlaceClassification *self = 
        (RendererPlaceClassification *) calloc (1, sizeof (RendererPlaceClassification));
    self->viewer = viewer;
    BotRenderer *r = &self->renderer;
    self->renderer.draw = _draw;
    self->renderer.destroy = _destroy;
    self->renderer.name = "Place Classification";
    self->renderer.user = self;
    self->renderer.enabled = 1;

    self->lcm = bot_lcm_get_global (NULL);

    //set up frame
    self->config = param;
    self->atrans = bot_frames_get_global(self->lcm, self->config);
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    //set up widget
    bot_gtk_param_widget_add_double(self->pw, "NO_LABEL",
                                    BOT_GTK_PARAM_WIDGET_SLIDER, 1, BIN_SIZE, 1, 10);
    bot_gtk_param_widget_add_double(self->pw, DISTANCE_TO_DRAW, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.1, DEFAULT_DIST);
    bot_gtk_param_widget_add_int(self->pw, DRAW_HISTORY_SIZE, BOT_GTK_PARAM_WIDGET_SLIDER, 0, BIN_SIZE, 1, BIN_SIZE);
    bot_gtk_param_widget_add_booleans(self->pw, 0, DRAW_TEXT, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, DRAW_PIE_CHART, 1, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, DRAW_MAX_LIKELIHOOD, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, DOORWAY_DETECTIONS, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, HMM_CLASSIFICATION, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, RAW_CLASSIFICATION, 1, NULL);

    bot_gtk_param_widget_add_booleans(self->pw, 0, COLOR_LINEAR_SCALE, 1, NULL);
    
    bot_gtk_param_widget_add_buttons(self->pw, CLEAR_HISTORY, NULL);

    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);

    self->renderer.widget = GTK_WIDGET(self->pw);

    //set up circular buffer
    self->classifications = bot_ptr_circular_new(BIN_SIZE, circ_free_classification_data, self);
    self->HMM_classifications = bot_ptr_circular_new(BIN_SIZE, circ_free_classification_data, self);

    self->label_sub = erlcm_place_classification_t_subscribe(self->lcm, "PLACE_CLASSIFICATION", on_classification, self);
    self->HMM_sub = erlcm_place_classification_t_subscribe(self->lcm, "HMM_PLACE_CLASSIFICATION", on_HMM, self);

    //set some default values
    self->no_label = 10;
    self->dist = DEFAULT_DIST;

    return r;
}

void setup_renderer_place_classification (BotViewer *viewer, int priority, BotParam *param)
{
    bot_viewer_add_renderer(viewer, 
                            renderer_place_classification_new(viewer, param), priority);
}

