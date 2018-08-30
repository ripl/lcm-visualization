#include "er_gl_utils.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
/*
 * er_gl_utils.h
 *
 * utility functions for drawing quad specific stuff - from Quad code base
 */

void draw_boxy_quad(const double pos[3],const double quat[4], const float body_color[3], const float front_color[3])
{
  // rotate and translate the vehicle
  glPushMatrix();
  double curr_quat_m[16];
  bot_quat_pos_to_matrix(quat, pos, curr_quat_m);
  // opengl expects column-major matrices
  double curr_quat_m_opengl[16];
  bot_matrix_transpose_4x4d(curr_quat_m, curr_quat_m_opengl);

  glMultMatrixd(curr_quat_m_opengl);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  //draw body
  glColor4f(body_color[0], body_color[1], body_color[2], .7);
  glPushMatrix();
  glScalef(.15, .8, .15);
  bot_gl_draw_cube();
  glPopMatrix();

  glPushMatrix();
  glScalef(.4, .15, .15);
  glTranslatef(-.4, 0, 0);
  bot_gl_draw_cube();
  glPopMatrix();

  //draw front
  glColor4f(front_color[0], front_color[1], front_color[2], .7);
  glPushMatrix();
  glScalef(.4, .15, .15);
  glTranslatef(0.4, 0, 0);
  bot_gl_draw_cube();
  glPopMatrix();

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  //pop matrix from translation
  glPopMatrix();
}
