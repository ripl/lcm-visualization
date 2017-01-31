
#include <stdio.h>
#include <glib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
//#include <GL/freeglut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
//#include <GL/freeglut.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gl_util.h>

#include <lcmtypes/bot_core_sensor_status_t.h>
#include <lcmtypes/bot_procman_info_t.h>
//#include <common/clock_skew_estimator.h>

#define ERR(fmt, ...) do {                                  \
      fprintf(stderr, "["__FILE__":%d Error: ", __LINE__);  \
      fprintf(stderr, fmt, ##__VA_ARGS__);                  \
  } while (0)

typedef struct
{
    char *id;    
    bot_core_sensor_status_t *sensor_status; 
} SS;

static void hs_destroy(SS *hs)
{
    free(hs->id);
    if(hs->sensor_status)
      bot_core_sensor_status_t_destroy(hs->sensor_status);
    free(hs);
}

static SS *hs_new(const char *name)
{
    SS *hs = (SS*)calloc(1, sizeof(SS));
    hs->id = g_strdup(name);
    hs->sensor_status = NULL;
    return hs;
}

static int hs_comp(const void *ptr_a, const void *ptr_b)
{
    SS *hs_a = *(SS**) ptr_a;
    SS *hs_b = *(SS**) ptr_b;
    return strcmp(hs_a->id, hs_b->id);
}


typedef struct
{
    BotRenderer renderer;
    BotViewer *viewer;

    lcm_t *lcm;
    bot_core_sensor_status_t_subscription_t *sub;
    bot_procman_info_t_subscription_t *pm_sub;

    // key: char*  val: SS*
    GHashTable *hosts;

    // Clock skew estimation
    //ClockSkewEstimator *skew;
    //char *ref_clock_name;
} RendererSensorStatus;

static SS * 
get_hs(RendererSensorStatus * self, const char *name)
{
    SS *hs = g_hash_table_lookup(self->hosts, name);
    if(!hs) {
        hs = hs_new(name);
        g_hash_table_insert(self->hosts, hs->id, hs);
    }

    return hs;
}

static void 
on_sensor_status(const lcm_recv_buf_t *rbuf, 
               const char *channel, const bot_core_sensor_status_t *msg, void *user)
{
    RendererSensorStatus *self = (RendererSensorStatus*)user;
    
    SS * hs = get_hs(self, msg->sensor_name);
    if(hs->sensor_status)
        bot_core_sensor_status_t_destroy(hs->sensor_status);
    hs->sensor_status = bot_core_sensor_status_t_copy(msg);
    bot_viewer_request_redraw(self->viewer);
}

////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Drawing Functions ------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void 
_draw(BotViewer *viewer, BotRenderer *r)
{
    RendererSensorStatus *self = (RendererSensorStatus*)r;

    GPtrArray * hosts = bot_g_hash_table_get_vals_array(self->hosts);
    if(!hosts->len) {
        g_ptr_array_free(hosts, TRUE);
        return;
    }
    g_ptr_array_sort(hosts, hs_comp); 

    glPushAttrib (GL_ENABLE_BIT);
    glEnable (GL_BLEND);
    glDisable (GL_DEPTH_TEST);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int gl_width = (GTK_WIDGET(viewer->gl_area)->allocation.width);
    int gl_height = (GTK_WIDGET(viewer->gl_area)->allocation.height);

    // transform into window coordinates, where <0, 0> is the top left corner
    // of the window and <viewport[2], viewport[3]> is the bottom right corner
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
    int line_height = 14;//14;

    float colors[3][3] = {
        { 0.6, 0.6, 0.6 },
        { 0.7, 0.7, 0.0 },
        { 1.0, 0.0, 0.0 },
    };

    //this is the no of sensors 
    int nhosts = hosts->len;
    //draws the box
    //double x = hind * 110 + 120;
    double x = gl_width - 20 * 8 ;//gl_width;
    //double y = gl_height - nhosts * line_height - 1;
    double y = line_height + 1;
    
    glColor4f(0, 0, 0, 0.7);
    glBegin(GL_QUADS);
    glVertex2f(x, y - line_height);
    glVertex2f(x + 20*8, y - line_height);
    glVertex2f(x + 20*8, y +  (nhosts + 1) * line_height);
    glVertex2f(x , y +  (nhosts + 1) * line_height);
    //glVertex2f(x, y + 5 * line_height);
    glEnd();
    
    char line2[80]; 

    sprintf(line2, "Sensor Rates (No :%d)", nhosts);

    glColor3fv(colors[0]);
    glRasterPos2f(x, y);
    bot_glutBitmapString(font, (unsigned char*) line2);

    for(int hind=0; hind<nhosts; hind++) {
        SS * hs = (SS*) g_ptr_array_index(hosts, hind);

        char *name = NULL; 
        double frequency = 0.0;

        if(hs->sensor_status) {
            name = hs->sensor_status->sensor_name;
            frequency = hs->sensor_status->rate; 
        }

        char line1[80];
        double y1 = (hind + 1) * line_height + y;

        sprintf(line1, "%s : %0.2fHz", name, frequency); 
        glColor3fv(colors[0]);
        glRasterPos2f(x, y1);
        bot_glutBitmapString(font, (unsigned char*) line1);

    }

    g_ptr_array_free(hosts, TRUE);

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

    RendererSensorStatus *self = (RendererSensorStatus*)r->user;
    if (self->lcm) {
        if (self->sub)
            bot_core_sensor_status_t_unsubscribe(self->lcm, self->sub);
    }
    if (self->hosts)
        g_hash_table_destroy(self->hosts);
    free(self);
}


BotRenderer *renderer_sensor_status_new(BotViewer *viewer)
{
    RendererSensorStatus *self = (RendererSensorStatus*)calloc(1, sizeof(RendererSensorStatus));
    self->viewer = viewer;

    BotRenderer *r = &self->renderer;
    self->renderer.name = "Sensor Status";
    self->renderer.draw = _draw;
    self->renderer.destroy = _destroy;

    self->hosts = g_hash_table_new_full(g_str_hash, g_str_equal,
                                        NULL, (GDestroyNotify) hs_destroy);

    self->lcm = bot_lcm_get_global (NULL);
    self->sub = bot_core_sensor_status_t_subscribe(self->lcm, "SENSOR_STATUS_.*", on_sensor_status, self);

    //we should subscribe to sensor status also 

    return r;
}

