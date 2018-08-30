/*
 * Renderer for tracks (moving objects)
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

#include <bot_core/bot_core.h>
#include <bot_vis/gtk_util.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gl_util.h>
#include <bot_param/param_client.h>

#include <bot_frames/bot_frames.h>

#define FORCE_RED

#include <lcmtypes/obs_track_list_t.h>
#include <lcmtypes/obs_obstacle_list_t.h>

#define RENDERER_NAME "Tracks"

struct track_info
{
    obs_track_list_t *tlist;
    double        color[3];

    char          channel[256];
};

typedef struct _RendererTracks {
    BotRenderer renderer;

    BotParam         *config;
    lcm_t           *lcm;
    GHashTable      *channels_hashtable;
    GPtrArray       *channels;
    BotViewer          *viewer;
    BotGtkParamWidget *pw;

  //**Needs to be bot trans 
  //ATrans          *atrans;
  BotFrames  *atrans;
    int64_t         last_utime;
} RendererTracks;


int frames_vehicle_pos_local(BotFrames *atrans, double pos[3])
{
    double pos_body[3] = { 0, 0, 0 };
    return bot_frames_transform_vec(atrans, "body", "local", pos_body, pos);
}

static void my_free( BotRenderer *renderer )
{
    RendererTracks *self = (RendererTracks*) renderer->user;

    free(self);
}

static void my_draw( BotViewer *viewer, BotRenderer *renderer )
{
    RendererTracks *self = (RendererTracks*) renderer->user;

    double carpos[3] = { 0, 0, 0 };
    frames_vehicle_pos_local(self->atrans, carpos);

    glLineWidth(1);
    glEnable(GL_BLEND);

    for (unsigned int chanidx = 0; chanidx < bot_g_ptr_array_size(self->channels); chanidx++) {
        struct track_info *tdata = g_ptr_array_index(self->channels, chanidx);

        obs_track_list_t *tlist = tdata->tlist;
	//this only draws the tracts - not static obs 
        for (int tidx = 0; tidx < tlist->ntracks; tidx++) {
            obs_track_t *t = &tlist->tracks[tidx];

            double x0 = t->pos[0];
            double y0 = t->pos[1];
            double sx = t->size[0]/2;
            double sy = t->size[1]/2;
#ifdef FORCE_RED
	    glColor4f(1,0,0,1);
#else
            glColor4d(tdata->color[0], tdata->color[1], tdata->color[2], 0.5);
#endif
            glLineWidth(2);
            double alpha = 1;
            glBegin(GL_LINES);
            glVertex3d(x0, y0, carpos[2]);
            glVertex3d(x0 + t->vel[0]*alpha, y0 + t->vel[1]*alpha, carpos[2]);
            glEnd();

            glPushMatrix();
            glTranslated(x0, y0, carpos[2]);
            glRotatef(bot_to_degrees(t->theta), 0, 0, 1);
#ifdef FORCE_RED
	    glColor4f(1,0,0,1);
#else
            glColor4d(tdata->color[0], tdata->color[1], tdata->color[2], 0.5);
#endif
            glBegin(GL_LINE_LOOP);
            glVertex2f( - sx, - sy);
            glVertex2f( - sx, + sy);
            glVertex2f( + sx, + sy);
            glVertex2f( + sx, - sy);
            glEnd();

            if (1) {
                char buf[128];
                double velocity = sqrt(bot_sq(t->vel[0]) + bot_sq(t->vel[1]));
                sprintf(buf, "%5.1f", velocity);
                double pos[3] = { 0, 0, 0 };
                glColor3f(0,0,0);
//                glutil_draw_text(pos, NULL, buf, GLUTIL_DRAW_TEXT_DROP_SHADOW);
                bot_gl_draw_text(pos, NULL, buf, 0);
            }
            
            glPopMatrix();
        }
    }
}

static void
on_tracks(const lcm_recv_buf_t *buf, const char *channel, 
	  const obs_track_list_t *msg, void *user_data )
{
    RendererTracks *self = (RendererTracks*) user_data;

    struct track_info *tinfo = g_hash_table_lookup(self->channels_hashtable, channel);
    if (tinfo == NULL) {
        tinfo = (struct track_info*) calloc(1, sizeof(struct track_info));
        strcpy(tinfo->channel, channel);
        g_hash_table_insert(self->channels_hashtable, tinfo->channel, tinfo);
        g_ptr_array_add(self->channels, tinfo);

        char key[256];
        sprintf(key, "%s.viewer_color", channel);
	printf("loading:%s\n",key);
        int sz = bot_param_get_double_array(self->config, key, tinfo->color, 3);
        if (sz != 3) {
            printf("%s : funny color!\n", key);
            tinfo->color[0] = 1;
            tinfo->color[1] = 1;
            tinfo->color[2] = 1;
        }
    }

    if (tinfo->tlist)
        obs_track_list_t_destroy(tinfo->tlist);

    tinfo->tlist = obs_track_list_t_copy(msg);

    bot_viewer_request_redraw(self->viewer);
}

static void
on_obstacles (const lcm_recv_buf_t *buf, const char * channel, 
	      const obs_obstacle_list_t * msg, void * user)
{
  on_tracks (buf, channel, &msg->tracks, user);
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererTracks *self = (RendererTracks*) user;

    bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererTracks *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererTracks *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

void setup_renderer_tracks(BotViewer *viewer, int priority, lcm_t *_lcm, BotParam * param) 
{
    RendererTracks *self = 
        (RendererTracks*) calloc(1, sizeof(RendererTracks));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = my_draw;
    renderer->destroy = my_free;
    renderer->name = RENDERER_NAME;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    self->lcm = _lcm; 
    self->config = param;
    self->atrans = bot_frames_get_global(self->lcm, self->config);//globals_get_atrans();
    
    self->channels_hashtable = g_hash_table_new(g_str_hash, g_str_equal);
    self->channels = g_ptr_array_new();
    self->viewer = viewer;

    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    
    obs_track_list_t_subscribe(self->lcm, ".*TRACKS.*", on_tracks, self);
    obs_obstacle_list_t_subscribe (self->lcm, "OBSTACLES", on_obstacles, self);

    bot_viewer_add_renderer(viewer, renderer, priority);

    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
            G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
            G_CALLBACK (on_save_preferences), self);

}
