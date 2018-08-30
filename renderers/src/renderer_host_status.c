
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
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_vis/gl_util.h>

#include <lcmtypes/ripl_host_status_t.h>
#include <lcmtypes/bot_procman_info2_t.h>
#include <hr_common/clock_skew_estimator.h>

#define ERR(fmt, ...) do {                                  \
      fprintf(stderr, "["__FILE__":%d Error: ", __LINE__);  \
      fprintf(stderr, fmt, ##__VA_ARGS__);                  \
  } while (0)

typedef struct
{
    char *id;
    ripl_host_status_t * host_status;
    bot_procman_info2_t * pmd_info;
} HS;

static void hs_destroy(HS *hs)
{
    free(hs->id);
    if(hs->host_status)
        ripl_host_status_t_destroy(hs->host_status);
    if(hs->pmd_info)
        bot_procman_info2_t_destroy(hs->pmd_info);
    free(hs);
}

static HS *hs_new(const char *name)
{
    HS *hs = (HS*)calloc(1, sizeof(HS));
    hs->id = g_strdup(name);
    hs->host_status = NULL;
    hs->pmd_info = NULL;
    return hs;
}

static int hs_comp(const void *ptr_a, const void *ptr_b)
{
    HS *hs_a = *(HS**) ptr_a;
    HS *hs_b = *(HS**) ptr_b;
    return strcmp(hs_a->id, hs_b->id);
}

typedef struct
{
    BotRenderer renderer;
    BotViewer *viewer;

    lcm_t *lcm;
    ripl_host_status_t_subscription_t *sub;
    bot_procman_info2_t_subscription_t *pm_sub;

    // key: char*  val: HS*
    GHashTable *hosts;

    // Clock skew estimation
    ClockSkewEstimator *skew;
    char *ref_clock_name;
} RendererHostStatus;

static HS *
get_hs(RendererHostStatus * self, const char *name)
{
    HS *hs = g_hash_table_lookup(self->hosts, name);
    if(!hs) {
        hs = hs_new(name);
        g_hash_table_insert(self->hosts, hs->id, hs);
    }
    return hs;
}

static void
on_host_status(const lcm_recv_buf_t *rbuf,
               const char *channel, const ripl_host_status_t *msg, void *user)
{
    RendererHostStatus *self = (RendererHostStatus*)user;
    HS * hs = get_hs(self, msg->id);
    if(hs->host_status)
        ripl_host_status_t_destroy(hs->host_status);
    hs->host_status = ripl_host_status_t_copy(msg);
    bot_viewer_request_redraw(self->viewer);
}

static void
on_procman_info_data(const lcm_recv_buf_t *rbuf,
                     const char *channel, const bot_procman_info2_t *msg, void *user)
{
    RendererHostStatus *self = (RendererHostStatus*) user;
    HS * hs = get_hs(self, msg->host);
    if(hs->pmd_info)
        bot_procman_info2_t_destroy(hs->pmd_info);
    hs->pmd_info = bot_procman_info2_t_copy(msg);

    clock_skew_estimator_add_measurement (self->skew, msg->host,
                                          rbuf->recv_utime, msg->utime);
    bot_viewer_request_redraw(self->viewer);
}

////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Drawing Functions ------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void
_draw(BotViewer *viewer, BotRenderer *r)
{
    RendererHostStatus *self = (RendererHostStatus*)r;

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
    int line_height = 14;

    float colors[3][3] = {
        { 0.6, 0.6, 0.6 },
        { 0.7, 0.7, 0.0 },
        { 1.0, 0.0, 0.0 },
    };

    int nhosts = hosts->len;
    for(int hind=0; hind<nhosts; hind++) {
        HS * hs = (HS*) g_ptr_array_index(hosts, hind);

        double batt = NAN;
        double tmp = NAN;
        int plugged_in = 1;
        double cpu = NAN;
        int64_t skew_int = -1;
        double skew_ms = NAN;
        double cur_cpu_freq_ghz = NAN;
        double max_cpu_freq_ghz = NAN;
        gboolean have_cpu_freq = FALSE;

        if(hs->host_status) {
            if(hs->host_status->nbatteries) {
                batt = hs->host_status->battery_charge[0] * 100;
            }
            tmp = hs->host_status->temp_C;
            plugged_in = hs->host_status->plugged_in;

            if (hs->host_status->num_processors) {
                int64_t min_cpu_freq = -1;
                int64_t max_cpu_freq = 1;

                for (int i=0; i<hs->host_status->num_processors; i++) {
                    if ((min_cpu_freq == -1) || (hs->host_status->cur_cpu_freq_hz[i] < min_cpu_freq)) {
                        min_cpu_freq = hs->host_status->cur_cpu_freq_hz[i];
                        max_cpu_freq = hs->host_status->max_cpu_freq_hz[i];
                    }
                }

                have_cpu_freq = TRUE;
                cur_cpu_freq_ghz = ((double) min_cpu_freq) * 1E-9;
                max_cpu_freq_ghz = ((double) max_cpu_freq) * 1E-9;
            }
        }
        if(hs->pmd_info) {
            cpu = hs->pmd_info->cpu_load * 100;
        }

        gboolean have_skew = clock_skew_estimator_get_skew (self->skew, hs->id,
                                                            &skew_int);
        if (strcmp(hs->id, self->ref_clock_name) == 0)
            skew_ms = 0;
        else {
            if(have_skew)
                skew_ms = skew_int * 1e-3;
        }

        // temperature
        int tmp_status = 0;
        if(tmp > 80) {
            tmp_status = 1;
        }
        if(tmp > 90) {
            tmp_status = 2;
        }

        // battery
        int bat_status = 0;

        if(batt < 70)
            bat_status = 1;
        if(batt < 40 || (!plugged_in))
            bat_status = 2;

        // CPU
        int cpu_status = 0;
        if(cpu > 80)
            cpu_status = 1;
        if(cpu > 90)
            cpu_status = 2;

        // host overall
        int host_status = MAX(tmp_status, bat_status);
        host_status = MAX(host_status, cpu_status);

        // clock skew
        int skew_status = 0;
        if (fabs(skew_ms) > 5)
            skew_status = 1;
        if (fabs(skew_ms) > 10)
            skew_status = 2;

        //        const char *line1 = hs->id;
        char line1[80], line2[80], line3[80], line4[80], line5[80], line6[80];

        if(hs->host_status) {
            sprintf(line1, "%8s (%2.0fC)", hs->id, tmp);
            sprintf(line2, "tmp:%4.1f C", tmp);
            sprintf(line3, "bat:%3.0f%% %s",
                    batt,
                    plugged_in == 1 ? "ACOK" : "NOAC");
        } else {
            sprintf(line1, "%8s", hs->id);
            sprintf(line2, "tmp: ????");
            sprintf(line3, "bat: ????");
        }
        if(hs->pmd_info) {
            sprintf(line4, "CPU:%4.1f%%", cpu);
        } else {
            sprintf(line4, "CPU: ????");
        }
        if(have_cpu_freq) {
            sprintf(line5, "%.2fGHz (%.2fGHz)", cur_cpu_freq_ghz, max_cpu_freq_ghz);
        } else {
            sprintf(line5, "cur: ????");
        }

        if (strcmp(hs->id, self->ref_clock_name)==0)
            sprintf(line6, "skew:0.0ms (ref)");
        else {
            if(have_skew)
                sprintf(line6, "skew:%4.2fms", skew_ms );
            else
                sprintf(line6, "skew: ????");
        }


        //double x = hind * 110 + 120;
        double x = hind * 175 + 10;
        double y = gl_height - 4 * line_height - 1;

        glColor4f(0, 0, 0, 0.7);
        glBegin(GL_QUADS);
        glVertex2f(x, y - line_height);
        glVertex2f(x + 20*8, y - line_height);
        //        glVertex2f(x + 13*8, y + 3 * line_height);
        //        glVertex2f(x, y + 3 * line_height);
        //glVertex2f(x + 13*8, y + 2 * line_height);
        //glVertex2f(x, y + 2 * line_height);
        glVertex2f(x + 20*8, y + 5 * line_height);
        glVertex2f(x, y + 5 * line_height);
        glEnd();

        glColor3fv(colors[host_status]);
        glRasterPos2f(x, y);
        bot_glutBitmapString(font, (unsigned char*) line1);

        //        glColor3fv(colors[tmp_status]);
        //        glRasterPos2f(x, y + 1 * line_height);
        //        bot_glutBitmapString(font, (unsigned char*) line2);

        glColor3fv(colors[bat_status]);
        glRasterPos2f(x, y + 1 * line_height);
        bot_glutBitmapString(font, (unsigned char*) line3);

        glColor3fv(colors[cpu_status]);
        glRasterPos2f(x, y + 2 * line_height);
        bot_glutBitmapString(font, (unsigned char*) line4);

        glColor3fv(colors[cpu_status]);
        glRasterPos2f(x, y + 3 * line_height);
        bot_glutBitmapString(font, (unsigned char*) line5);

        glColor3fv(colors[skew_status]);
        glRasterPos2f(x, y + 4 * line_height);
        bot_glutBitmapString(font, (unsigned char*) line6);
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

    RendererHostStatus *self = (RendererHostStatus*)r->user;
    if (self->lcm) {
        if (self->sub)
            ripl_host_status_t_unsubscribe(self->lcm, self->sub);
    }
    if (self->hosts)
        g_hash_table_destroy(self->hosts);
    free(self);
}

BotRenderer *renderer_host_status_new(BotViewer *viewer)
{
    RendererHostStatus *self = (RendererHostStatus*)calloc(1, sizeof(RendererHostStatus));
    self->viewer = viewer;

    BotRenderer *r = &self->renderer;
    self->renderer.name = "Host Status";
    self->renderer.draw = _draw;
    self->renderer.destroy = _destroy;

    self->hosts = g_hash_table_new_full(g_str_hash, g_str_equal,
                                        NULL, (GDestroyNotify) hs_destroy);

    // setup clock skew estimator using mwalter-agile as reference
    self->ref_clock_name = strdup("laptop.husky");
    self->skew = clock_skew_estimator_new (self->ref_clock_name);

    self->lcm = bot_lcm_get_global (NULL);
    self->sub = ripl_host_status_t_subscribe(self->lcm, "HOST_STATUS", on_host_status, self);

    self->pm_sub = bot_procman_info2_t_subscribe(self->lcm, "PMD_INFO2", on_procman_info_data, self);

    //we should subscribe to sensor status also

    return r;
}

void setup_renderer_host_status(BotViewer *viewer, int priority)
{
    BotRenderer *r = renderer_host_status_new(viewer);
    bot_viewer_add_renderer(viewer, r, priority);
}
