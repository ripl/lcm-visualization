
#include <stdio.h>
#include <glib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gl_util.h>

#include <lcmtypes/ripl_annotation_t.h>

#define ERR(fmt, ...) do {                                  \
      fprintf(stderr, "["__FILE__":%d Error: ", __LINE__);  \
      fprintf(stderr, fmt, ##__VA_ARGS__);                  \
  } while (0)


typedef struct
{
    BotRenderer renderer;
    BotViewer *viewer;

    lcm_t *lcm;

    ripl_annotation_t_subscription_t *sub;
  
    ripl_annotation_t *annotation_last;

} RendererAnnotation;




static void 
on_annotation(const lcm_recv_buf_t *rbuf, 
              const char *channel, const ripl_annotation_t *msg, void *user)
{
    RendererAnnotation *self = (RendererAnnotation*)user;

    if (self->annotation_last)
        ripl_annotation_t_destroy (self->annotation_last);

    self->annotation_last = ripl_annotation_t_copy(msg);

    bot_viewer_request_redraw(self->viewer);
}


////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Drawing Functions ------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void 
_draw(BotViewer *viewer, BotRenderer *r)
{
    RendererAnnotation *self = (RendererAnnotation*)r;

    if (!self->annotation_last)
        return;

    glPushAttrib (GL_ENABLE_BIT);
    glEnable (GL_BLEND);
    glDisable (GL_DEPTH_TEST);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int gl_width = (GTK_WIDGET(viewer->gl_area)->allocation.width);
    int gl_height = (GTK_WIDGET(viewer->gl_area)->allocation.height);

    // transform into window coordinates, where <0, 0> is the top left corner
    // of the window and <gl_width, gl_height> is the bottom right corner
    // of the window
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, gl_width, 0, gl_height);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0, gl_height, 0);
    glScalef(1, -1, 1);

    void *font = GLUT_BITMAP_8_BY_13;
    int line_height = 14;

    float colors[3][3] = {
        { 0.6, 0.6, 0.6 },
        { 0.7, 0.7, 0.0 },
        { 1.0, 0.0, 0.0 },
    };


    // Form the string that you'd like to print to the viewer.
    // This could just be the annotation from the message or you could include
    // the audio channel name, for example "[HEADSET_AUDIO]: The kitchen is down the hall"
    char annotation_line[256];
     sprintf (annotation_line, "[%s]: %s", self->annotation_last->reference_channel, self->annotation_last->annotation);
    // sprintf (annotation_line, "%s", self->annotation_last->annotation);

    //double x = hind * 110 + 120;
//    double line_height = 14;
    double x = 10;
    double y = gl_height - 4 * line_height - 1;
    

    glColor4f(0, 0, 0, 0.7);
    glBegin(GL_QUADS);
    glVertex2f(x, y - line_height);
    glVertex2f(x + 20*8, y - line_height);
    glVertex2f(x + 20*8, y + 5 * line_height);
    glVertex2f(x, y + 5 * line_height);
    glEnd();

    // Use bot_glutBitmapString() to draw annotation string at (x, y)
    glRasterPos2f(x, y);
    bot_glutBitmapString(font, (unsigned char*) annotation_line);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib ();
}


////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Up and Down ------------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void 
_destroy(BotRenderer *r)
{
    if (!r) return;

    RendererAnnotation *self = (RendererAnnotation*)r->user;
    free(self);
}

BotRenderer *renderer_annotation_new(BotViewer *viewer)
{
    RendererAnnotation *self = (RendererAnnotation*)calloc(1, sizeof(RendererAnnotation));
    self->viewer = viewer;

    // Defining functions that take care of drawing on the screen and destroying
    BotRenderer *r = &self->renderer;
    self->renderer.name = "Annotation";
    self->renderer.draw = _draw;
    self->renderer.destroy = _destroy;

    // Get the handle to LCM, which is necessary to subscribe to messages
    self->lcm = bot_lcm_get_global (NULL);

    // Subscribe to the annotation message 
    self->sub = ripl_annotation_t_subscribe(self->lcm, "ANNOTATION", on_annotation, self);

    return r;
}

void setup_renderer_annotation(BotViewer *viewer, int priority)
{
    BotRenderer *r = renderer_annotation_new(viewer);
    bot_viewer_add_renderer(viewer, r, priority);
}
