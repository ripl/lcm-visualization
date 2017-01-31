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

#include <bot_core/bot_core.h>
#include <bot_vis/gl_util.h>
#include <bot_vis/viewer.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

//#include <carmen3d/lcm_utils.h>
//#include <carmen3d/carmen3d_common.h>

#define RENDERER_NAME "ManualCalib"

#define PARAM_SENSOR_CHOICE "Sensors"
#define PARAM_X         "X    "
#define PARAM_Y         "Y    "
#define PARAM_Z         "Z    "
#define PARAM_ROLL      "Roll "
#define PARAM_PITCH     "Pitch"
#define PARAM_YAW       "Yaw  "

#define PARAM_SAVE  "Save Calib"

typedef struct _RendererManualCalib RendererManualCalib;

struct _RendererManualCalib {
    lcm_t *lcm;

    BotViewer *viewer;
    BotRenderer renderer;
    BotEventHandler ehandler;
   
    BotParam * param;
    BotFrames * frames;

    BotGtkParamWidget *pw;
    int numSensors;
    char ** sensorNames;
    char ** sensorRelativeTo;
    int * sensorNums;

    int activeSensorNum;
    //BotCTransLink * sensorLink;
    BotTrans sensorLink;
    int sensorSetupDone;
};


static void draw_axis(BotTrans * frame_to_local){

    double frame_to_local_m[16];
    bot_trans_get_mat_4x4(frame_to_local,frame_to_local_m);

    // opengl expects column-major matrices
    double frame_to_local_m_opengl[16];
    bot_matrix_transpose_4x4d(frame_to_local_m, frame_to_local_m_opengl);

    glPushMatrix();
    // rotate and translate the vehicle
    glMultMatrixd(frame_to_local_m_opengl);

    glEnable(GL_DEPTH_TEST);

    //draw an axis
    glPointSize(5);
    glLineWidth(3);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(.2, 0, 0);
    glEnd();
    glColor3f(0, 1, 0);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, .2, 0);
    glEnd();
    glColor3f(0, 0, 1);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, .2);
    glEnd();

    glPopMatrix();
}

static void draw_frame(BotFrames *frames, const char * frame)
{

    // compute the rotation matrix to orient the vehicle in world coordinates
    BotTrans frame_to_local;
    bot_frames_get_trans(frames, frame, "local", &frame_to_local);
    draw_axis(&frame_to_local);
}

static void draw_link(BotFrames *frames, const char * from, const char * to)
{

    BotTrans fromPos;

    bot_frames_get_trans(frames, from, "body", &fromPos);

    BotTrans toPos;
    bot_frames_get_trans(frames, to, "local", &toPos);
    glPointSize(5);
    glLineWidth(5);
    glColor3f(1, 1, 0);
    glBegin(GL_LINES);
    glVertex3dv(fromPos.trans_vec);
    glVertex3dv(toPos.trans_vec);
    glEnd();

}

static void draw(BotViewer *viewer, BotRenderer *renderer)
{
    RendererManualCalib *self = (RendererManualCalib*) renderer->user;

    if (self->activeSensorNum <= 0)
        return;

    draw_link(self->frames, self->sensorNames[self->activeSensorNum], "body");

}

static int key_press(BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event)
{
    //  RendererManualCalib *self = (RendererManualCalib*) ehandler->user;
    return 0;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *param_name, void *user)
{
    RendererManualCalib *self = (RendererManualCalib*) user;
    if (!strcmp(param_name, PARAM_SENSOR_CHOICE)) {
        self->activeSensorNum = bot_gtk_param_widget_get_enum(pw, PARAM_SENSOR_CHOICE);
        if (self->activeSensorNum > 0) {
            self->sensorSetupDone=0;
            bot_viewer_set_status_bar_message(self->viewer, "Modify Calibration relative to %s",
                                          self->sensorRelativeTo[self->activeSensorNum]);
            BotTrans tran;
            if(!bot_frames_get_trans( self->frames, self->sensorNames[self->activeSensorNum],
                                     self->sensorRelativeTo[self->activeSensorNum], &tran)) {
                fprintf (stdout, "Error getting transformation from %s to %s via bot_frames\n",
                         self->sensorNames[self->activeSensorNum], self->sensorRelativeTo[self->activeSensorNum]);
                return;
            }

            double rpy[3];
            bot_quat_to_roll_pitch_yaw(tran.rot_quat, rpy);

            bot_gtk_param_widget_set_double(self->pw, PARAM_X, tran.trans_vec[0]);
            bot_gtk_param_widget_set_double(self->pw, PARAM_Y, tran.trans_vec[1]);
            bot_gtk_param_widget_set_double(self->pw, PARAM_Z, tran.trans_vec[2]);
            bot_gtk_param_widget_set_double(self->pw, PARAM_ROLL, bot_to_degrees(rpy[0]));
            bot_gtk_param_widget_set_double(self->pw, PARAM_PITCH, bot_to_degrees(rpy[1]));
            bot_gtk_param_widget_set_double(self->pw, PARAM_YAW, bot_to_degrees(rpy[2]));

            bot_gtk_param_widget_set_enabled(self->pw, PARAM_X, 1);
            bot_gtk_param_widget_set_enabled(self->pw, PARAM_Y, 1);
            bot_gtk_param_widget_set_enabled(self->pw, PARAM_Z, 1);
            bot_gtk_param_widget_set_enabled(self->pw, PARAM_ROLL, 1);
            bot_gtk_param_widget_set_enabled(self->pw, PARAM_PITCH, 1);
            bot_gtk_param_widget_set_enabled(self->pw, PARAM_YAW, 1);
            bot_gtk_param_widget_set_enabled(self->pw, PARAM_SAVE, 1);

            BotTrans test;
            if(!bot_frames_get_trans(self->frames, self->sensorNames[self->activeSensorNum],
                            self->sensorRelativeTo[self->activeSensorNum], &test))//self->sensorLink))
                fprintf (stdout, "Error getting bot_frames trans from %s to %s\n",
                         self->sensorNames[self->activeSensorNum], self->sensorRelativeTo[self->activeSensorNum]);
            else
                self->sensorSetupDone=1;

        }
    }
    if (!strcmp(param_name, PARAM_SAVE)) {
        char save_fname[1024];
        sprintf(save_fname, "manual_calib_%s.cfg", self->sensorNames[self->activeSensorNum]);
        fprintf(stderr, "saving params to: %s\n", save_fname);
        FILE * f = fopen(save_fname, "w");
        double pos[3];
        pos[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_X);
        pos[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_Y);
        pos[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_Z);
        double rpy[3];
        rpy[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_ROLL);
        rpy[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_PITCH);
        rpy[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_YAW);

        double rpy_rad[3];
        for (int i=0;i<3;i++)
            rpy_rad[i] = bot_to_radians(rpy[i]);
        double quat[4];
        bot_roll_pitch_yaw_to_quat(rpy_rad,quat);
        double rod[3];
        bot_quat_to_rodrigues(quat,rod);

        fprintf(f, ""
                "%s {\n"
                "position = [%f, %f, %f];\n"
                "rpy = [%f, %f, %f];\n"
                "relative_to = \"%s\";\n"
                "}", self->sensorNames[self->activeSensorNum], pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2],
                self->sensorRelativeTo[self->activeSensorNum]);
        fprintf(f, ""
                "\n"
                "%s {\n"
                "position = [%f, %f, %f];\n"
                "rodrigues = [%f, %f, %f];\n"
                "relative_to = \"%s\";\n"
                "}", self->sensorNames[self->activeSensorNum], pos[0], pos[1], pos[2], rod[0], rod[1], rod[2],
                self->sensorRelativeTo[self->activeSensorNum]);


        fclose(f);
        bot_viewer_set_status_bar_message(self->viewer, "Calibration saved to %s", save_fname);

    }
    else if (self->activeSensorNum > 0 && self->sensorSetupDone) { //sensorLink is set last...
        BotTrans curr;
        curr.trans_vec[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_X);
        curr.trans_vec[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_Y);
        curr.trans_vec[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_Z);

        double rpy[3];
        rpy[0] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_ROLL));
        rpy[1] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_PITCH));
        rpy[2] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_YAW));
        bot_roll_pitch_yaw_to_quat(rpy, curr.rot_quat);
        if (fabs(rpy[0]) > M_PI || fabs(rpy[1]) > M_PI || fabs(rpy[2]) > M_PI) {
            bot_gtk_param_widget_set_double(self->pw, PARAM_ROLL, bot_to_degrees(bot_mod2pi(rpy[0])));
            bot_gtk_param_widget_set_double(self->pw, PARAM_PITCH, bot_to_degrees(bot_mod2pi(rpy[1])));
            bot_gtk_param_widget_set_double(self->pw, PARAM_YAW, bot_to_degrees(bot_mod2pi(rpy[2])));
        }

        //and update the link
        //bot_ctrans_link_update(self->sensorLink, &curr, bot_timestamp_now());

        int64_t latest_trans_timestamp;
        bot_frames_get_trans_latest_timestamp(self->frames, self->sensorNames[self->activeSensorNum],
                                              self->sensorRelativeTo[self->activeSensorNum], &latest_trans_timestamp);

        //int64_t now = bot_timestamp_now();
        bot_frames_update_frame (self->frames, self->sensorNames[self->activeSensorNum],
                self->sensorRelativeTo[self->activeSensorNum], &curr, latest_trans_timestamp);

    }

    if (self->activeSensorNum <= 0) {
        bot_viewer_set_status_bar_message(self->viewer, "");
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_X, 0);
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_Y, 0);
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_Z, 0);
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_ROLL, 0);
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_PITCH, 0);
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_YAW, 0);
        bot_gtk_param_widget_set_enabled(self->pw, PARAM_SAVE, 0);
    }

    bot_viewer_request_redraw(self->viewer);
}

static void destroy(BotRenderer *super)
{
    RendererManualCalib *self = (RendererManualCalib*) super->user;

    if (!self)
        return;

    if (self->sensorNames)
        free (self->sensorNames);
    if (self->sensorRelativeTo)
        free (self->sensorRelativeTo);
    if (self->sensorNums)
        free (self->sensorNums);

    free(self);
}

BotRenderer *renderer_manual_calib_new(BotViewer *viewer, int render_priority)
{
    RendererManualCalib *self = (RendererManualCalib*) calloc(1, sizeof(RendererManualCalib));
    self->viewer = viewer;
    self->renderer.draw = draw;
    self->renderer.destroy = destroy;
    self->renderer.name = RENDERER_NAME;
    self->renderer.user = self;
    self->renderer.enabled = 1;

    self->lcm = bot_lcm_get_global (NULL);

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->key_press = key_press;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = NULL;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;
    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());


    if (!(self->param = bot_param_get_global (self->lcm, 0))) {
        fprintf (stderr, "No param server found\n");
        return 0;
    }

    if (!(self->frames = bot_frames_get_global (self->lcm, self->param))) {
        fprintf (stderr, "Error getting bot_frames\n");
        return 0;
    }

    //bot_glib_mainloop_attach_lcm (self->lcm);


    int numLidars = 0;
    int numCams = 0;
    int numVelodynes = 0;
    int numRGBDCams = 0;

    // get the planar lidar (SICK, hokuyo) coordinate names
    char **planar_lidar_names = bot_param_get_all_planar_lidar_names(self->param);
    if (!planar_lidar_names)
        fprintf(stderr, "["__FILE__":%d] Error: Could not get lidar names.\n", __LINE__);
    else {
        for (int pind = 0; planar_lidar_names[pind] != NULL; pind++)
            numLidars++;
    }

    // get the camera coordinate names
    char **camnames = bot_param_get_all_camera_names(self->param);
    if (!camnames)
        fprintf(stderr, "["__FILE__":%d] Error: Could not get camera names.\n", __LINE__);
    else {
        for (int cind = 0; camnames[cind] != NULL; cind++)
            numCams++;
    }

    char **velodyne_names = bot_param_get_subkeys (self->param, "three_dimensional_lidars");
    if (!velodyne_names)
        fprintf(stderr, "["__FILE__":%d] Error: Could not get velodyne names.\n", __LINE__);
    else {
        for (int vind = 0; velodyne_names[vind] != NULL; vind++)
            numVelodynes++;
    }

    char **rgbd_names = bot_param_get_subkeys (self->param, "rgbd_cameras");
    if (!rgbd_names)
        fprintf(stderr, "["__FILE__":%d] Error: Could not get kinect names.\n", __LINE__);
    else {
        for (int kind = 0; rgbd_names[kind] != NULL; kind++)
            numRGBDCams++;
    }

    self->numSensors = numLidars + numCams + numVelodynes + numRGBDCams + 1; //+1 cuz blank first one
    self->sensorNames = calloc(self->numSensors + 1, sizeof(char*)); //+1 cuz of NULL last one
    self->sensorRelativeTo = calloc(self->numSensors + 1, sizeof(char*)); //+1 cuz of NULL last one
    self->sensorNums = calloc(self->numSensors, sizeof(int));

    int sind = 0;
    self->sensorNames[sind++] = "  "; //blank first one...

    if (planar_lidar_names) {
        for (int pind = 0; planar_lidar_names[pind]; pind++) {
            self->sensorNums[sind] = sind;
            self->sensorNames[sind] = strdup(planar_lidar_names[pind]);
            
            char calib_key[1024];
            sprintf(calib_key, "planar_lidars.%s.coord_frame", planar_lidar_names[pind]);

            char *coord_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &coord_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }

            sprintf(calib_key, "coordinate_frames.%s.relative_to", coord_frame);
            char *ref_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &ref_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }
            self->sensorRelativeTo[sind] = strdup(ref_frame);
            sind++;

            free(coord_frame);
            free(ref_frame);
        }
    }

    if (camnames) {
        for (int cind = 0; camnames[cind]; cind++) {
            self->sensorNums[sind] = sind;
            self->sensorNames[sind] = strdup(camnames[cind]);
            char calib_key[1024];
            sprintf(calib_key, "cameras.%s.coord_frame", camnames[cind]);

            char *coord_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &coord_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }

            sprintf(calib_key, "coordinate_frames.%s.relative_to", coord_frame);
            char *ref_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &ref_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }
            self->sensorRelativeTo[sind] = strdup(ref_frame);
            sind++;

            free(coord_frame);
            free(ref_frame);
        }
    }

    
    if (velodyne_names) {
        for (int vind = 0; velodyne_names[vind]; vind++) {
            self->sensorNums[sind] = sind;
            self->sensorNames[sind] = strdup(velodyne_names[vind]);
            char calib_key[1024];
            sprintf(calib_key, "three_dimensional_lidars.%s.coord_frame", velodyne_names[vind]);

            char *coord_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &coord_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }

            sprintf(calib_key, "coordinate_frames.%s.relative_to", coord_frame);
            char *ref_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &ref_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }
            self->sensorRelativeTo[sind] = strdup(ref_frame);
            sind++;

            free(coord_frame);
            free(ref_frame);
        }
    }

    if (rgbd_names) {
        for (int kind = 0; rgbd_names[kind]; kind++) {
            self->sensorNums[sind] = sind;
            self->sensorNames[sind] = strdup(rgbd_names[kind]);
            char calib_key[1024];

            sprintf(calib_key, "rgbd_cameras.%s.coord_frame", rgbd_names[kind]);

            char *coord_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &coord_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }

            sprintf(calib_key, "coordinate_frames.%s.relative_to", coord_frame);
            char *ref_frame;
            if (0 != bot_param_get_str(self->param, calib_key, &ref_frame)) {
                fprintf (stderr, "Error reading %s for param server\n", calib_key);
                return 0;
            }
            self->sensorRelativeTo[sind] = strdup(ref_frame);
            sind++;

            free(coord_frame);
            free(ref_frame);
        }
    }

    g_strfreev(camnames);
    g_strfreev(planar_lidar_names);
    g_strfreev(velodyne_names);
    g_strfreev(rgbd_names);

    bot_gtk_param_widget_add_enumv(self->pw, PARAM_SENSOR_CHOICE, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, self->numSensors,
            (const char **) self->sensorNames, self->sensorNums);
    bot_gtk_param_widget_add_double(self->pw, PARAM_X, BOT_GTK_PARAM_WIDGET_SPINBOX, -8, 8, 0.001, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_Y, BOT_GTK_PARAM_WIDGET_SPINBOX, -8, 8, 0.001, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, -8, 8, 0.001, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_ROLL, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_PITCH, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_YAW, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);

    bot_gtk_param_widget_add_separator(self->pw, "");
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SAVE, NULL);

    bot_gtk_param_widget_set_enabled(self->pw, PARAM_X, 0);
    bot_gtk_param_widget_set_enabled(self->pw, PARAM_Y, 0);
    bot_gtk_param_widget_set_enabled(self->pw, PARAM_Z, 0);
    bot_gtk_param_widget_set_enabled(self->pw, PARAM_ROLL, 0);
    bot_gtk_param_widget_set_enabled(self->pw, PARAM_PITCH, 0);
    bot_gtk_param_widget_set_enabled(self->pw, PARAM_YAW, 0);
    bot_gtk_param_widget_set_enabled(self->pw, PARAM_SAVE, 0);

    /* setup signal callbacks */
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->renderer.widget = GTK_WIDGET(self->pw);

    self->activeSensorNum = 0;
    self->sensorSetupDone = 1;

    return &self->renderer;
}

void setup_renderer_manual_calib(BotViewer *viewer, int render_priority)
{
    bot_viewer_add_renderer(viewer, renderer_manual_calib_new(viewer, render_priority), render_priority);
}
