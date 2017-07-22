/*
 * Rendering birds-eye satellite view
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dirent.h>

#include <GL/gl.h>
#include <GL/glu.h>


#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <hr_common/path_util.h>
#include "er_gl_utils.h"

#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/bot_core_pose_t.h>

// #include <lcm/lcm.h>
// #include <bot/bot_core.h>
// #include <bot/gtk/gtk_util.h>
// #include <bot/gl/gl_util.h>
// #include <bot/gl/texture.h>
// #include <bot/viewer/viewer.h>
// #include <bot/viewer/rwx.h>
// #include <bot/core/fasttrig.h>

//#include <common/gridmap.h>
//#include <common/globals.h>
#include <image_utils/jpeg.h>
//#include <common/math_util.h>
//#include <common/model_config.h>

//#include "tile_set.h"


#define RENDERER_NAME "SatelliteView"

#define MAX_TILE_AGE 1

#define SAT_TILE_WIDTH 256
#define SAT_TILE_HEIGHT 256
#define MAX_SATELLITE_CAR_DIST 30000 // max distance between a satellite tile and the car, in meters

#define PARAM_SATELLITE_IMAGE "Satellite image"
#define PARAM_NAME_SAT_OPACITY "Satellite opacity"
#define PARAM_NAME_SATELLITE_XOFF "Satellite x offset"
#define PARAM_NAME_SATELLITE_YOFF "Satellite y offset"

typedef struct _satellite_tile { double x0, y0, x1, y1; BotGlTexture    *tex; } satellite_tile_t;

typedef struct _RendererSatelliteView {
    BotRenderer         renderer;

    GMutex * mutex;

    BotParam           *param;
    BotGPSLinearize    *gps_linearize;
    lcm_t              *lcm;
    GHashTable         *channels_hashtable;
    GPtrArray          *channels;
    BotViewer             *viewer;
    BotGtkParamWidget  *pw;
    BotFrames          *frames;
    unsigned char      *sat_data;   // satellite image data
    GQueue             *satellite_tiles;

    erlcm_gps_to_local_t gps_to_local;
    int have_gps_to_local;

    erlcm_gps_to_local_t_subscription_t * gps_subscr;

    //int have_gps_linearize_at_global_origin;


  } RendererSatelliteView;



int have_gps_to_local (RendererSatelliteView *self)
{
    g_mutex_lock (self->mutex);
    int result = self->have_gps_to_local;
    g_mutex_unlock (self->mutex);
    return result;
}

/* gps is (lat, lon, elevation) */
/* returns 0 on error, 1 on success */
int
gps_to_local (RendererSatelliteView *self, const double gps[3], double xyz[3], double cov[3][3])
{
    if (!have_gps_to_local (self))
    return 0;
    g_mutex_lock (self->mutex);
    double p_g[2];
    bot_gps_linearize_to_xy (self->gps_linearize, gps, p_g);

    double theta = self->gps_to_local.lat_lon_el_theta[3];
    double sine, cosine;
    #ifdef sincos
    sincos (theta, &sine, &cosine);
    #else
    sine = sin(theta);
    cosine = cos(theta);
    #endif

    xyz[0] = p_g[0] * cosine - p_g[1] * sine;
    xyz[1] = p_g[0] * sine   + p_g[1] * cosine;
    xyz[2] = gps[2];

    // now add the local frame offset
    xyz[0] += self->gps_to_local.local[0];
    xyz[1] += self->gps_to_local.local[1];
    xyz[2] += self->gps_to_local.local[2];

    //if (cov) {
    //    fprintf (stderr, "TODO: gps_to_local covariance computation\n");
    //}

    g_mutex_unlock (self->mutex);
    return 1;
  }




static void
_latlon_local_pos (RendererSatelliteView *self, double lat, double lon, double p[3])
{
    double gps[3] = { lat, lon, 0 };
    gps_to_local(self, gps, p, NULL);
}

/* load satellite images into a set of textures */
void load_satellite_images( RendererSatelliteView *self, const char *dirname )
{
    // open the directory
    DIR *dirp = opendir( dirname );
    if ( dirp == NULL ) {
        printf("Error reading directory %s\n", dirname);
        return;
    }

    struct dirent  *dp;

    // for each image in the directory...
    while ((dp = readdir(dirp)) != NULL) {

        int id1,id2,id3,id4,id5,id6,id7,id8;

        // read lat and lon from filename
        if ( sscanf(dp->d_name, "%d.%d.%d.%d.%d.%d.%d.%d.jpg", &id1, &id2, &id3, &id4, \
                    &id5, &id6, &id7, &id8) != 8 )
            continue;

        // convert lat-lon to x-y coordinates
        double topleft_lat = (double)id1 + (double)id2 / 1000000;
        double topleft_lon = - (double)id3 - (double)id4 / 1000000;
        double bottomright_lat = (double)id5 + (double)id6 / 1000000;
        double bottomright_lon = - (double)id7 - (double)id8 / 1000000;

        // create a new satellite tile structure
        satellite_tile_t *tt = (satellite_tile_t*)malloc(sizeof(satellite_tile_t));

        tt->x0 = topleft_lat;
        tt->y0 = topleft_lon;
        tt->x1 = bottomright_lat;
        tt->y1 = bottomright_lon;
        tt->tex = bot_gl_texture_new (SAT_TILE_WIDTH, SAT_TILE_HEIGHT, 3 *SAT_TILE_WIDTH*SAT_TILE_HEIGHT);

        // load the file into the texture
        char fullname[512];
        sprintf(fullname, "%s/%s", dirname, dp->d_name);

        FILE *fp = fopen(fullname,"r");
        if ( !fp )
            continue;

        printf("Loading %s\n", fullname);

        // obtain file size
        fseek (fp , 0 , SEEK_END);
        int fp_size = ftell (fp);
        rewind (fp);

        // read the file
        uint8_t * src = NULL;

        if (fp && fp_size > 0) {
            src = (uint8_t*)malloc(fp_size);
            fread( src, 1, fp_size, fp);
        }

        if (fp)
            fclose(fp);

        int dest_size = 3*SAT_TILE_WIDTH*SAT_TILE_HEIGHT*sizeof(uint8_t);
        uint8_t *data = (uint8_t*)calloc(dest_size, 1);

        // uncompress the jpeg image into data
        if (src) {
            int jres = jpeg_decompress_8u_rgb(src, fp_size, data,
                    SAT_TILE_WIDTH,
                    SAT_TILE_HEIGHT, 3*SAT_TILE_WIDTH);
            assert(jres==0);
            bot_gl_texture_upload( tt->tex, GL_RGB, GL_UNSIGNED_BYTE,
                    3*SAT_TILE_WIDTH, data);
            free(src);
        }

        free(data);

        // append the satellite tile to the global array

        g_queue_push_head (self->satellite_tiles, tt);
    }

    closedir( dirp );

    printf ("Done.\n");
}



static void
on_gps_to_local (const lcm_recv_buf_t * rbuf, const char * channel,
                 const erlcm_gps_to_local_t * gl, void * user)
{
    RendererSatelliteView * self = (RendererSatelliteView *) user;
    g_mutex_lock(self->mutex);

    memcpy (&self->gps_to_local, gl, sizeof (erlcm_gps_to_local_t));
    bot_gps_linearize_init(self->gps_linearize, gl->lat_lon_el_theta);

    self->have_gps_to_local = 1;

    g_mutex_unlock (self->mutex);
}


// static gboolean tileset_remove_helper(gpointer k, gpointer v, gpointer user)
// {
//     tile_set_destroy(v);
//     return TRUE;
// }

static void my_free( BotRenderer *renderer )
{
    RendererSatelliteView *self = (RendererSatelliteView*) renderer->user;

    // unload the satellite tiles
    for (GList *iter=g_queue_peek_head_link (self->satellite_tiles);iter;iter=iter->next) {
        satellite_tile_t *tt = (satellite_tile_t*)iter->data;
        if (tt->tex)
            bot_gl_texture_free (tt->tex);
        free (tt);
    }
    g_queue_clear (self->satellite_tiles);

    free( self );
}


static void my_draw( BotViewer *viewer, BotRenderer *renderer )
{
    RendererSatelliteView *self = (RendererSatelliteView*) renderer->user;

    double pos_body[3] = { 0, 0, 0 };
    double pos[3] = {0, 0, 0};
    bot_frames_transform_vec (self->frames, "body", "local", pos_body, pos);

    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable (GL_COLOR_MATERIAL);

    // draw the satellite images
    if (viewer && bot_gtk_param_widget_get_bool(self->pw, PARAM_SATELLITE_IMAGE)) {
        glColor4d (1, 1, 1,
                bot_gtk_param_widget_get_double (self->pw, PARAM_NAME_SAT_OPACITY));

        double xoffset = bot_gtk_param_widget_get_double(self->pw,
                PARAM_NAME_SATELLITE_XOFF);
        double yoffset = bot_gtk_param_widget_get_double(self->pw,
                PARAM_NAME_SATELLITE_YOFF);

        for (GList *iter=g_queue_peek_head_link (self->satellite_tiles);iter;iter=iter->next) {
            satellite_tile_t *tt = (satellite_tile_t*)iter->data;

            double topleft[3];
            double topright[3];
            double bottomleft[3];
            double bottomright[3];

            _latlon_local_pos( self, tt->x0, tt->y0, topleft );
            _latlon_local_pos( self, tt->x1, tt->y1, bottomright );
            _latlon_local_pos( self, tt->x0, tt->y1, bottomleft );
            _latlon_local_pos( self, tt->x1, tt->y0, topright );

            double cx = (topleft[0]+bottomright[0])/2;
            double cy = (topleft[1]+bottomleft[1])/2;
            if (fabs(cx-pos[0])+fabs(cy-pos[1]) < MAX_SATELLITE_CAR_DIST)
                bot_gl_texture_draw_coords (tt->tex,
                        topleft[0]+xoffset, topleft[1]+yoffset, 0,
                        topright[0]+xoffset, topright[1]+yoffset, 0,
                        bottomright[0]+xoffset, bottomright[1]+yoffset, 0,
                        bottomleft[0]+xoffset, bottomleft[1]+yoffset, 0);
        }
    }


    glDisable(GL_COLOR_MATERIAL);

}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererSatelliteView *self = (RendererSatelliteView*) user;
    (void) self;

    bot_viewer_request_redraw(self->viewer);
}

void on_load_sat_activate(GtkMenuItem *mi, void *user)
{
    RendererSatelliteView *self = (RendererSatelliteView*) user;


    if (g_queue_is_empty (self->satellite_tiles)) {

        // prompt user for a directory containing the satellite images

        GtkWidget *dialog;

        dialog = gtk_file_chooser_dialog_new ("Select directory for satellite images",
                NULL,
                GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER,
                GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
                NULL);

        const char * google_maps_path = getSatellitePath();
        gtk_file_chooser_set_current_folder (GTK_FILE_CHOOSER (dialog),
                google_maps_path);

        if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
            char *dirname;
            dirname = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
            if ( dirname )
                load_satellite_images( self, dirname );
            g_free (dirname);
        }

        gtk_widget_destroy (dialog);

    }  else {

        printf("Freeing %d textures\n", g_queue_get_length (self->satellite_tiles));

        // unload the satellite tiles
        for (GList *iter=g_queue_peek_head_link (self->satellite_tiles);iter;iter=iter->next) {
            satellite_tile_t *tt = (satellite_tile_t*)iter->data;
            if (tt->tex) bot_gl_texture_free( tt->tex );
            free (tt);
        }

        g_queue_clear (self->satellite_tiles);

    }
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererSatelliteView *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);

    // always deactivate satellite underlay
    bot_gtk_param_widget_set_bool( self->pw, PARAM_SATELLITE_IMAGE, 0 );
}

    static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererSatelliteView *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

BotRenderer *renderer_satellite_new(BotViewer *viewer, BotParam *param, BotFrames *frames)
{
    RendererSatelliteView *self =
        (RendererSatelliteView*) calloc(1, sizeof(RendererSatelliteView));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = my_draw;
    renderer->destroy = my_free;
    renderer->name = RENDERER_NAME;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    self->lcm = bot_lcm_get_global(NULL);
    self->frames = frames;
    self->param = param;;
    //self->channels_hashtable = g_hash_table_new(g_str_hash, g_str_equal);
    //self->channels = g_ptr_array_new();
    self->viewer = viewer;
    self->satellite_tiles = g_queue_new ();//g_array_new(FALSE, FALSE, sizeof(satellite_tile_t));

    if (viewer)
    {
        bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_SATELLITE_IMAGE, 0, NULL);
        bot_gtk_param_widget_add_double (self->pw,
                PARAM_NAME_SATELLITE_XOFF,
                BOT_GTK_PARAM_WIDGET_SLIDER, -10, 10, .1, 0);
        bot_gtk_param_widget_add_double (self->pw,
                PARAM_NAME_SATELLITE_YOFF,
                BOT_GTK_PARAM_WIDGET_SLIDER, -10, 10, .1, 0);

        bot_gtk_param_widget_add_double (self->pw,
                PARAM_NAME_SAT_OPACITY,
                BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.05, 1);

        // Load
        GtkWidget *load_sat = gtk_menu_item_new_with_mnemonic ("_Load/Unload satellite images...");
        gtk_menu_append (GTK_MENU(viewer->file_menu), load_sat);
        gtk_widget_show (load_sat);
        g_signal_connect (G_OBJECT (load_sat), "activate",
                G_CALLBACK (on_load_sat_activate), self);

        g_signal_connect (G_OBJECT (self->pw), "changed",
                G_CALLBACK (on_param_widget_changed), self);
    }


    ///////////////////////////////////////////////////////
    // obstacle hazard map
#define SIZE 256
    if (1) {
        float r[SIZE];
        float g[SIZE];
        float b[SIZE];
        float a[SIZE];

        for (int i = 0; i < SIZE; i++) {
            double f = ((float) i) / 255;
            double scale = 1.0;
            r[i] = f * scale;
            g[i] = f * scale;
            b[i] = f * scale;
            a[i] = 1;

            if (i & 1) {
                r[i] = 0;
                g[i] = 0;
            }
        }

        r[SIZE-1] = 1.0;
        g[SIZE-1] = .3;
        b[SIZE-1] = .3;
        r[SIZE-2] = 1.0;
        g[SIZE-2] = .3;
        b[SIZE-2] = .3;
    }

    ///////////////////////////////////////////////////////
    // hazard color map
    if (1) {
        float r[SIZE];
        float g[SIZE];
        float b[SIZE];
        float a[SIZE];

        for (int i = 0; i < SIZE; i++) {
            double f = ((float) i) / 255;
            double scale = 0.8;
            r[i] = f * scale;
            g[i] = 0;
            b[i] = 0;
            a[i] = .5;
        }

        g[1] = .5;
        b[1] = .5;

    }
    ///////////////////////////////////////////////////////
    // speed color map
    if (1) {
        float r[SIZE];
        float g[SIZE];
        float b[SIZE];
        float a[SIZE];

        for (int i = 0; i < SIZE; i++) {
            double f = ((float) i) / 255.0;
            r[i] = fmin(2.0*f, 1.0);
            g[i] = fmax(fmin(2.0-(2.0*f),1.0),0.0);
            b[i] = 0;
            a[i] = .6;
        }

        //g[1] = .5;
        //b[1] = .5;

    }
#undef SIZE

    erlcm_gps_to_local_t_subscribe(self->lcm, "GPS_TO_LOCAL", on_gps_to_local, self);

    return renderer;
}

void setup_renderer_satellite(BotViewer *viewer, int render_priority, BotParam *param, BotFrames *frames)
{
    BotRenderer *renderer = renderer_satellite_new(viewer, param, frames);
    if (viewer)
    {
        bot_viewer_add_renderer(viewer, renderer, render_priority);

        g_signal_connect (G_OBJECT (viewer), "load-preferences",
                G_CALLBACK (on_load_preferences), renderer);
        g_signal_connect (G_OBJECT (viewer), "save-preferences",
                G_CALLBACK (on_save_preferences), renderer);
    }
}
