#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <GL/gl.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <hr_common/path_util.h>
#include "er_gl_utils.h"

#include <lcmtypes/hr_lcmtypes.h>

#define QUAD_COORD_FRAME "body"

#define RENDERER_NAME "Quad"

#define PARAM_BLING "Bling"
#define PARAM_SHOW_SHADOW "Show Shadow"

typedef struct _RendererQuad {
  BotRenderer renderer;
  BotEventHandler ehandler;

  lcm_t *lcm;
  BotParam * param;
  BotFrames * frames;

  BotRwxModel *quad_model;
  BotViewer *viewer;
  BotGtkParamWidget *pw;

  const char * draw_frame;
  const char * model_param_prefix;

  int display_lists_ready;
  GLuint quad_dl;
} RendererQuad;

static void frames_update_handler(BotFrames *bot_frames, const char *frame, const char * relative_to, int64_t utime,
    void *user)
{
  RendererQuad *self = (RendererQuad *) user;
  if (strcmp(frame, QUAD_COORD_FRAME) == 0)
    bot_viewer_request_redraw(self->viewer);
}

static void on_find_button(GtkWidget *button, RendererQuad *self)
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

static void quad_free(BotRenderer *super)
{
  RendererQuad *self = (RendererQuad*) super->user;

  if (self->quad_model)
    bot_rwx_model_destroy(self->quad_model);
  free(self);
}

static GLuint compile_rwx_display_list(RendererQuad * self, BotRwxModel * model)
{
  GLuint dl = glGenLists(1);
  glNewList(dl, GL_COMPILE);

  glPushMatrix();

  const char * prefix = self->model_param_prefix;
  char key[1024];
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

  sprintf(key, "%s.translate", prefix);
  double trans[3];
  if (bot_param_get_double_array(self->param, key, trans, 3) == 3)
    glTranslated(trans[0], trans[1], trans[2]);

  glEnable(GL_BLEND);
  glEnable(GL_RESCALE_NORMAL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);

  glEnable(GL_LIGHTING);
  bot_rwx_model_gl_draw(model);
  glDisable(GL_LIGHTING);
  glDisable(GL_RESCALE_NORMAL);
  glDisable(GL_BLEND);

  glPopMatrix();
  glEndList();
  return dl;
}

static void quad_draw(BotViewer *viewer, BotRenderer *super)
{
  RendererQuad *self = (RendererQuad*) super->user;

  if (!bot_frames_have_trans(self->frames, QUAD_COORD_FRAME, self->draw_frame))
    return;

  int bling = bot_gtk_param_widget_get_bool(self->pw, PARAM_BLING);
  if (bling && self->quad_model && !self->display_lists_ready) {
    self->quad_dl = compile_rwx_display_list(self, self->quad_model);
    self->display_lists_ready = 1;
  }

  // get the transform to orient the vehicle in drawing coordinates
  BotTrans body_to_local;
  bot_frames_get_trans(self->frames, QUAD_COORD_FRAME, self->draw_frame, &body_to_local);

  if (bling && self->display_lists_ready) {
    double body_to_local_m[16], body_to_local_m_opengl[16];
    bot_trans_get_mat_4x4(&body_to_local, body_to_local_m);
    bot_matrix_transpose_4x4d(body_to_local_m, body_to_local_m_opengl);// opengl expects column-major matrices
    glPushMatrix();
    glMultMatrixd(body_to_local_m_opengl); // rotate and translate the vehicle
    glCallList(self->quad_dl); //draw the rwx model from the display_lists
    glPopMatrix();

  }
  else {
    draw_boxy_quad(body_to_local.trans_vec, body_to_local.rot_quat, bot_color_util_blue, bot_color_util_red);
  }

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
  }
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererQuad *self = (RendererQuad*) user;
  bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererQuad *self = (RendererQuad *) user_data;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererQuad *self = (RendererQuad *) user_data;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

void add_quad_model_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param, BotFrames * frames)
{
  RendererQuad *self = (RendererQuad*) calloc(1, sizeof(RendererQuad));

  BotRenderer *renderer = &self->renderer;

  renderer->draw = quad_draw;
  renderer->destroy = quad_free;

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
  /* attempt to load RWX model files */
  self->param = param;
  self->frames = frames;
  bot_frames_add_update_subscriber(self->frames, frames_update_handler, (void *) self);

  self->draw_frame = bot_frames_get_root_name(self->frames);

  const char * models_dir = getModelsPath();

  
  char *model_name;
  char model_full_path[256];
  self->model_param_prefix = "models.quad";
  char param_key[1024];
  snprintf(param_key, sizeof(param_key), "%s.model", self->model_param_prefix);
  if (bot_param_get_str(self->param, param_key, &model_name) == 0) {
    snprintf(model_full_path, sizeof(model_full_path), "%s/%s", models_dir, model_name);
    self->quad_model = bot_rwx_model_create(model_full_path);
  }
  else {
    fprintf(stderr, "Quad model name not found under param %s, drawing with boxy-quad\n", param_key);
  }

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_BLING, 1, PARAM_SHOW_SHADOW, 0, NULL);

  GtkWidget *find_button = gtk_button_new_with_label("Find");
  gtk_box_pack_start(GTK_BOX(renderer->widget), find_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(find_button), "clicked", G_CALLBACK(on_find_button), self);

  gtk_widget_show_all(renderer->widget);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  on_param_widget_changed(self->pw, "", self);

  bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
  //    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);


  g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
  g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
}
