#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <path_utils/path_util.h>
#include "er_gl_utils.h"
#include <GL/glut.h>

#include <lcmtypes/hr_lcmtypes.h>

#define QUAD_COORD_FRAME "body"

#define RENDERER_NAME "Person"

#define PARAM_BLING "Bling"
#define PARAM_DRAW_STATUS "Draw Status"
#define PARAM_SHOW_SHADOW "Show Shadow"

typedef enum {FOLLOWING, WAITING, STOPPED} following_status_t;

typedef struct _RendererPerson {
    BotRenderer renderer;
    BotEventHandler ehandler;

    lcm_t *lcm;
    BotParam * param;
    BotFrames * frames;

    BotWavefrontModel *person_model;
    BotViewer *viewer;
    BotGtkParamWidget *pw;

    ripl_people_pos_msg_t *people_pos_list;

    const char * draw_frame;
    const char * model_param_prefix;

    following_status_t following_state;
    int draw_status;
    int display_lists_ready;
    GLuint person_dl;
    GMutex *mutex;
} RendererPerson;

static void
on_person_follower(const lcm_recv_buf_t *buf, const char *channel,
              const ripl_speech_cmd_t *msg, void *user) {
    
    RendererPerson *self = (RendererPerson*)user;
    //fprintf(stderr, "%s -> %s\n", msg->cmd_type, msg->cmd_property);
    if(!strcmp(msg->cmd_type, "FOLLOWER")){
        if(!strcmp(msg->cmd_property, "STOPPED")){
            self->following_state = STOPPED;
        }
        else if(!strcmp(msg->cmd_property, "FOLLOWING")){
            self->following_state = FOLLOWING;
        }
        else if(!strcmp(msg->cmd_property, "WAITING")){
            self->following_state = WAITING;
        }        
    }
    //g_mutex_unlock (self->mutex);
  
    bot_viewer_request_redraw(self->viewer);

}

static void
on_people_pos(const lcm_recv_buf_t *buf, const char *channel,
              const ripl_people_pos_msg_t *msg, void *user) {
    
    RendererPerson *self = (RendererPerson*)user;

    // g_mutex_lock (self->mutex);
  
    if (self->people_pos_list) 
        ripl_people_pos_msg_t_destroy (self->people_pos_list);
    self->people_pos_list = ripl_people_pos_msg_t_copy (msg);
  
    //g_mutex_unlock (self->mutex);
  
    bot_viewer_request_redraw(self->viewer);

}


static void
draw_wavefront_model (RendererPerson * self, double x, double y, double heading)
{
    //printf("drawing model %f %f %f \n",x, y, heading);
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);

    glPushMatrix();
    ///x,y,z
    glTranslated(x, y, 0);
    
    //orientation
    glRotatef(heading, 0, 0, 1);

    glCallList (self->person_dl);

    //undo 
    glPopMatrix();
    //orientation
    //glRotatef(-heading, 0, 0, 1);
    ///x,y,z
    //glTranslated(-x, -y, 0);
    
    
}

static void frames_update_handler(BotFrames *bot_frames, const char *frame, const char * relative_to, int64_t utime,
                                  void *user)
{
    RendererPerson *self = (RendererPerson *) user;
    if (strcmp(frame, QUAD_COORD_FRAME) == 0)
        bot_viewer_request_redraw(self->viewer);
}

static void on_find_button(GtkWidget *button, RendererPerson *self)
{
    BotViewHandler *vhandler = self->viewer->view_handler;

    double eye[3];
    double lookat[3];
    double up[3];

    vhandler->get_eye_look(vhandler, eye, lookat, up);
    double diff[3];
    bot_vector_subtract_3d(eye, lookat, diff);

    BotTrans pose;
    bot_frames_get_trans(self->frames, QUAD_COORD_FRAME, self->draw_frame, &pose);
    bot_vector_add_3d(pose.trans_vec, diff, eye);

    vhandler->set_look_at(vhandler, eye, pose.trans_vec, up);

    bot_viewer_request_redraw(self->viewer);
}

static void person_free(BotRenderer *super)
{
    RendererPerson *self = (RendererPerson*) super->user;

    if (self->person_model)
        bot_wavefront_model_destroy(self->person_model);
    free(self);
}

static GLuint
compile_display_list (RendererPerson * self, BotWavefrontModel * model)
{
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);
    
    const char * prefix = self->model_param_prefix;
    char key[1024];

    glPushMatrix();

    sprintf(key, "%s.translate", prefix);
    double trans[3];
    if (bot_param_get_double_array(self->param, key, trans, 3) == 3)
        glTranslated(trans[0], trans[1], trans[2]);

    sprintf(key, "%s.scale", prefix);
    double scale;
    if (bot_param_get_double(self->param, key, &scale) == 0)
        glScalef(scale, scale, scale);

    sprintf(key, "%s.rotate_xyz", prefix);
    double rot[3];
    if (bot_param_get_double_array(self->param, key, rot, 3) == 3) {
        glRotatef(rot[2], 0, 0, 1);
        glRotatef(rot[1], 0, 1, 0);
        glRotatef(rot[0], 1, 0, 0);
    }

    glEnable(GL_LIGHTING);
    bot_wavefront_model_gl_draw(model);
    glDisable(GL_LIGHTING);

    glPopMatrix();

    glEndList ();
    return dl;
}


static void person_draw(BotViewer *viewer, BotRenderer *super)
{
    RendererPerson *self = (RendererPerson*) super->user;

    ///////////////////////////////////////////////////////////////////////////
    /* for now, use this to test. take it out when messages are being published*/
    ///////////////////////////////////////////////////////////////////////////
    /*self->people_pos_list = (ripl_people_pos_msg_t*)calloc(1, sizeof(ripl_people_pos_msg_t));
      self->people_pos_list->num_people = 1;
      ripl_person_status_t* person_status_list  = (ripl_person_status_t*)calloc(1, sizeof(ripl_person_status_t));
      person_status_list->pos[0] = -1;
      person_status_list->pos[1] = 1.5;
      person_status_list->person_heading = -45;
      self->people_pos_list->people_pos = person_status_list;*/
    ///////////////////////////////////////////////////////////////////////////

    if (!bot_frames_have_trans(self->frames, QUAD_COORD_FRAME, self->draw_frame))
        return;

    int bling = bot_gtk_param_widget_get_bool(self->pw, PARAM_BLING);
    if (bling && self->person_model && !self->display_lists_ready) {
        self->person_dl = compile_display_list(self, self->person_model);
        self->display_lists_ready = 1;
    }

    // get the transform to orient the vehicle in drawing coordinates
    BotTrans body_to_local;
    bot_frames_get_trans(self->frames, QUAD_COORD_FRAME, self->draw_frame, &body_to_local);
 
    if(self->draw_status){
        double pos[] = {150, 90, 100};
        
        GLint viewport[4];
        glGetIntegerv (GL_VIEWPORT, viewport);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0, viewport[2], 0, viewport[3]);

        glColor3f(1,1,1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glColor3f(1.0,1.0,1.0);
        
        char name[1020];
        
        if(self->people_pos_list == NULL){
            sprintf(name, "Person Tracker Inactive");
        }
        else if(self->people_pos_list->num_people == 0){
            sprintf(name, "No Person seen");
        }
        else if(self->people_pos_list->followed_person != -1){
            //char following_state[200]; 

            switch(self->following_state){
            case STOPPED:
                //sprintf(following_state, "STOPPED");
                sprintf(name, "Stopped");
                break;
            case FOLLOWING:
                sprintf(name, "Following");
                //sprintf(following_state, "FOLLOWING");
                break;
            case WAITING:
                sprintf(name, "Waiting");
                //sprintf(following_state, "WAITING");
                break;
            default:
                sprintf(name, "Error");
                //sprintf(following_state, "Error");
                break;
            }
            //sprintf(name, "Tracking Person");
            
        }
        else{
            sprintf(name, "Looking");
        }

        //fprintf(stderr, "Following state : %s\n", name);
        
        //bot_gl_draw_text(pos, GLUT_BITMAP_TIMES_ROMAN_24, name,
        bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_18, name,
                         BOT_GL_DRAW_TEXT_JUSTIFY_LEFT |
                         BOT_GL_DRAW_TEXT_DROP_SHADOW |
                         BOT_GL_DRAW_TEXT_ANCHOR_RIGHT);

        glLoadIdentity();

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }
  

    if (bling && self->display_lists_ready 
        && self->people_pos_list != NULL) { 

        double body_to_local_m[16], body_to_local_m_opengl[16];

        glEnable(GL_DEPTH_TEST);

        bot_trans_get_mat_4x4(&body_to_local, body_to_local_m);
        bot_matrix_transpose_4x4d(body_to_local_m, body_to_local_m_opengl);// opengl expects column-major matrices
        glPushMatrix();
        glMultMatrixd(body_to_local_m_opengl); // rotate and translate the vehicle
    
        // draw each individual
        for (int i = 0; i < self->people_pos_list->num_people; i ++) {
            //if(self->people_pos_list->followed_person >=0 && i == self->people_pos_list->followed_person){
            ripl_person_status_t person = self->people_pos_list->people_pos[i];     
            draw_wavefront_model(self, person.pos[0], person.pos[1], bot_to_degrees(person.person_heading));
            //}
        }

        glPopMatrix();

    }
    //else {
    //    draw_boxy_quad(body_to_local.trans_vec, body_to_local.rot_quat, bot_color_util_blue, bot_color_util_red);
    //}
    /*
      if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_SHADOW)) {
      glColor4f(0, 0, 0, 0.2);
      glPushMatrix();
      glTranslated(body_to_local.trans_vec[0], body_to_local.trans_vec[1], 0);
      glTranslated(-0.04, 0.03, 0);
      double rpy[3];
      bot_quat_to_roll_pitch_yaw(body_to_local.rot_quat, rpy);
      glRotatef(rpy[2] * 180 / M_PI + 45, 0, 0, 1);

      glEnable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBegin(GL_QUADS);
      glVertex2f(0.3, -0.3);
      glVertex2f(0.3, 0.3);
      glVertex2f(-0.3, 0.3);
      glVertex2f(-0.3, -0.3);
      glEnd();
      glPopMatrix();
      glDisable(GL_BLEND);
      glDisable(GL_DEPTH_TEST);
      }*/

    ///////////////////////////////////////////////////////////////////////////
    /* for now, use this to test. take it out when messages are being published*/
    ///////////////////////////////////////////////////////////////////////////
    //free(person_status_list);
    //free(self->people_pos_list);
    ///////////////////////////////////////////////////////////////////////////
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererPerson *self = (RendererPerson*) user;
    self->draw_status = bot_gtk_param_widget_get_bool (self->pw, PARAM_DRAW_STATUS);
    bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPerson *self = (RendererPerson *) user_data;
    bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererPerson *self = (RendererPerson *) user_data;
    bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

void add_person_model_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param, BotFrames * frames, lcm_t *_lcm)
{
    RendererPerson *self = (RendererPerson*) calloc(1, sizeof(RendererPerson));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = person_draw;
    renderer->destroy = person_free;

    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char *) RENDERER_NAME;
    ehandler->enabled = 0;
    ehandler->pick_query = NULL;
    ehandler->key_press = NULL;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = NULL;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    self->viewer = viewer;
    /* attempt to load wavefront model files */
    self->param = param;
    self->frames = frames;
    self->people_pos_list = NULL;

    self->lcm = _lcm;
    //self->followed_person = 0;

    bot_frames_add_update_subscriber(self->frames, frames_update_handler, (void *) self);

    ripl_people_pos_msg_t_subscribe (self->lcm, "PEOPLE_LIST", on_people_pos, self);
    ripl_speech_cmd_t_subscribe(self->lcm, "PERSON_TRACKER", on_person_follower, self);
    self->draw_frame = bot_frames_get_root_name(self->frames);

    const char * models_dir = getModelsPath();

    char *model_name;
    char model_full_path[256];
    self->model_param_prefix = "models.person";
    char param_key[1024];
    snprintf(param_key, sizeof(param_key), "%s.wavefront_model", self->model_param_prefix);
  
    if (bot_param_get_str(self->param, param_key, &model_name) == 0) {
        snprintf(model_full_path, sizeof(model_full_path), "%s/%s", models_dir, model_name);
        //fprintf(stderr, "Full path : %s\n" , model_full_path);
        self->person_model = bot_wavefront_model_create(model_full_path);
        double minv[3];
        double maxv[3];
        bot_wavefront_model_get_extrema(self->person_model, minv, maxv);

        double span_x = maxv[0] - minv[0];
        double span_y = maxv[1] - minv[1];
        double span_z = maxv[2] - minv[2];

        double span_max = MAX(span_x, MAX(span_y, span_z));

        //printf("WAVEFRONT extrema: [%f, %f, %f] [%f, %f, %f]\n", 
	//   minv[0], minv[1], minv[2],
	//   maxv[0], maxv[1], maxv[2]);

    }
    else {
        fprintf(stderr, "person model name not found under param %s, drawing with boxy-robot\n", param_key);
    }


    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);

    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_BLING, 1, PARAM_SHOW_SHADOW, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,   PARAM_DRAW_STATUS, 1, NULL);

    GtkWidget *find_button = gtk_button_new_with_label("Find");
    gtk_box_pack_start(GTK_BOX(renderer->widget), find_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(find_button), "clicked", G_CALLBACK(on_find_button), self);

    gtk_widget_show_all(renderer->widget);

    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    on_param_widget_changed(self->pw, "", self);

    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    //    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    self->following_state = STOPPED;
    self->draw_status = 1;

    g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
    g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
}
