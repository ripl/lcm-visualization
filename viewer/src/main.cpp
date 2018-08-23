#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <assert.h>
#include <getopt.h>


#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <hr_common/path_util.h>
#include <lcmtypes/erlcm_robot_state_command_t.h>
#include <lcmtypes/erlcm_shift_velocity_msg_t.h>
#include <lcmtypes/erlcm_robot_status_t.h>


// Renderers
#include <bot_frames/bot_frames_renderers.h>
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <er_renderers/er_renderers.h>
#include <er_renderers/viewer_aux_data.h>
//#include <kinect/kinect_renderer.h>
#include <laser_utils/renderer_laser.h>
#include <image_utils/renderer_cam_thumb.h>
#include <velodyne/renderer_velodyne.h>
#include <occ_map/occ_map_renderers.h>
//#include <octomap_utils/renderer_octomap.h>

//#include <visualization/collections_renderer.hpp>

#include "udp_util.h"

static BotCamTrans* global_camtrans;

// convenience function to get local pose
int frames_vehicle_pose_local (BotFrames *frames, bot_core_pose_t *pose)
{
    BotTrans body_to_local;
    int ret = bot_frames_get_trans (frames, "body", "local", &body_to_local);

    if (ret) {
        memcpy (pose->pos, body_to_local.trans_vec, 3 * sizeof(double));
        memcpy (pose->orientation, body_to_local.rot_quat, 4 * sizeof (double));
    }

    return ret;
}

static int
robot_state_on_key_press(BotViewer *viewer, BotEventHandler *ehandler,
                         const GdkEventKey *event)
{
    int keyval = event->keyval;

    const ViewerAuxData * aux_data = get_viewer_aux_data (viewer);
    if (!aux_data->lcm)
        return 0;

    erlcm_robot_state_command_t cmd;
    erlcm_shift_velocity_msg_t shift_cmd;

    switch (keyval) {
    case 'F':
        // clear faults
        cmd.utime = bot_timestamp_now();
        cmd.state = ERLCM_ROBOT_STATUS_T_STATE_STOP;
        cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_NONE;
        cmd.fault_mask = ERLCM_ROBOT_STATUS_T_FAULT_MASK_CLEAR_ALL;
        cmd.sender = "viewer";
        cmd.comment = "Clear Faults";
        erlcm_robot_state_command_t_publish(aux_data->lcm,"ROBOT_STATE_COMMAND",&cmd);
        break;
    case 'R':
        // go into run mode
        cmd.utime = bot_timestamp_now();
        cmd.state = ERLCM_ROBOT_STATUS_T_STATE_RUN;
        cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_NONE;
        cmd.fault_mask = ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
        cmd.sender = "viewer";
        cmd.comment = "Activating";
        erlcm_robot_state_command_t_publish(aux_data->lcm,"ROBOT_STATE_COMMAND",&cmd);

        break;
    case ' ':
        // go into pause mode eating key press.
        cmd.utime = bot_timestamp_now();
        cmd.state = ERLCM_ROBOT_STATUS_T_STATE_STOP;
        cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_NONE;
        cmd.fault_mask = ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
        cmd.sender = "viewer";
        cmd.comment = "Stopping";
        erlcm_robot_state_command_t_publish(aux_data->lcm,"ROBOT_STATE_COMMAND",&cmd);

        break;
    case 'P':
    case 'p':
        // go into pause mode
        // don't eat keypress let logplayer see it too.
        cmd.utime = bot_timestamp_now();
        cmd.state = ERLCM_ROBOT_STATUS_T_STATE_STOP;
        cmd.faults = ERLCM_ROBOT_STATUS_T_FAULT_NONE;
        cmd.fault_mask = ERLCM_ROBOT_STATUS_T_FAULT_MASK_NO_CHANGE;
        cmd.sender = "viewer";
        cmd.comment = "Pause requested.";
        erlcm_robot_state_command_t_publish(aux_data->lcm,"ROBOT_STATE_COMMAND",&cmd);
        return 0;
        break;
    case 'Q':
    case 'q':
        //W is speed up
        shift_cmd.utime = bot_timestamp_now();
        shift_cmd.shift = ERLCM_SHIFT_VELOCITY_MSG_T_SHIFT_UP;
        erlcm_shift_velocity_msg_t_publish(aux_data->lcm,"SHIFT_VELOCITY_CMD",&shift_cmd);
        return 0;
        break;
    case 'W':
    case 'w':

        //slow the robot down
        shift_cmd.utime = bot_timestamp_now();
        shift_cmd.shift = ERLCM_SHIFT_VELOCITY_MSG_T_SHIFT_DOWN;
        erlcm_shift_velocity_msg_t_publish(aux_data->lcm,"SHIFT_VELOCITY_CMD",&shift_cmd);
        return 0;

        break;

    default:
        return 0;
    }

    return 1;
}

static int
logplayer_remote_on_key_press(BotViewer *viewer, BotEventHandler *ehandler,
                              const GdkEventKey *event)
{
    int keyval = event->keyval;

    const ViewerAuxData * aux_data = get_viewer_aux_data (viewer);
    if (!aux_data->logplayer_udp_port)
        return 0;

    switch (keyval) {
    case 'P':
    case 'p':
        // go into pause mode.
        udp_send_string("127.0.0.1", aux_data->logplayer_udp_port, "PLAYPAUSETOGGLE");
        break;
    case 'N':
    case 'n':
        udp_send_string("127.0.0.1", aux_data->logplayer_udp_port, "STEP");
        break;
    case '=':
    case '+':
        udp_send_string("127.0.0.1", aux_data->logplayer_udp_port, "FASTER");
        break;
    case '_':
    case '-':
        udp_send_string("127.0.0.1", aux_data->logplayer_udp_port, "SLOWER");
        break;
    case '[':
        udp_send_string("127.0.0.1", aux_data->logplayer_udp_port, "BACK5");
        break;
    case ']':
        udp_send_string("127.0.0.1", aux_data->logplayer_udp_port, "FORWARD5");
        break;
    default:
        return 0;
    }

    return 1;
}

static inline double
get_cam_vertical_fov (BotCamTrans *camtrans)
{
    double cop_x = bot_camtrans_get_principal_x (camtrans);
    double cop_y = bot_camtrans_get_principal_y (camtrans);
    double cam_height = bot_camtrans_get_image_height (camtrans);

    double upper[3], lower[3], middle[3];
    bot_camtrans_unproject_pixel(camtrans, cop_x, 0, upper);
    bot_camtrans_unproject_pixel(camtrans, cop_x, cop_y, middle);
    bot_camtrans_unproject_pixel(camtrans, cop_x, cam_height, lower);

    // since the center of projection may not be in the middle of the image,
    // the angle from the COP to the top pixel may not be the same as the angle
    // from the COP to the bottom pixel.  Set the FOV to be twice the
    // smaller of these two angles, so that the entire vertical FOV is within
    // the image extents
    double theta1 = bot_vector_angle_3d (upper, middle);
    double theta2 = bot_vector_angle_3d (lower, middle);
    double fov = MIN (theta1, theta2) * 2;
    return fov;
}

static void
setup_vhandler_with_camview(ViewerAuxData * aux, const bot_core_pose_t *msg)
{
    if(!aux->active_camtrans)
        return;
    BotViewHandler *vhandler = aux->viewer->view_handler;
    double look_at_cam[3] = { 0, 0, 1 }, look_at_local[3];
    double up_cam[3] = { 0, -1, 0 }, up_local[3];

    const char *cam_name = bot_camtrans_get_name(aux->active_camtrans);
    BotTrans cam_to_local;
    if(!bot_frames_get_trans (aux->frames, cam_name, "local", &cam_to_local))
        return;
    bot_trans_apply_vec (&cam_to_local, look_at_cam, look_at_local);
    bot_trans_rotate_vec (&cam_to_local, up_cam, up_local);
    const double *cam_pos_local = cam_to_local.trans_vec;

    vhandler->set_look_at(vhandler, cam_pos_local, look_at_local, up_local);
}

static void
on_camera_view_handler_rmi_activate (GtkRadioMenuItem *rmi, void *user_data)
{
    if(! gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(rmi)))
        return;
    ViewerAuxData *aux = (ViewerAuxData*) user_data;
    BotCamTrans *camtrans = (BotCamTrans*) g_object_get_data (G_OBJECT (rmi), "camtrans");
    aux->active_camtrans = camtrans;

    double fov_y = get_cam_vertical_fov (aux->active_camtrans) * 180 / M_PI;;

    BotViewHandler *vhandler = aux->viewer->view_handler;
    vhandler->set_camera_perspective(vhandler, fov_y);

    bot_core_pose_t pose;
    frames_vehicle_pose_local (aux->frames, &pose);
    setup_vhandler_with_camview(aux, &pose);

    bot_viewer_request_redraw(aux->viewer);
}

static void
on_pose (const lcm_recv_buf_t *rbuf, const char *channel,
         const bot_core_pose_t *msg, void *user_data)
{
    ViewerAuxData *self = (ViewerAuxData*) user_data;
    setup_vhandler_with_camview (self, msg);
}



/////////////////////////////////////////////////////////////

static void on_perspective_item(GtkMenuItem *mi, void *user)
{
    ViewerAuxData *aux = (ViewerAuxData*)user;
    BotViewHandler *vhandler = aux->viewer->view_handler;
    if(vhandler) {
        aux->active_camtrans = NULL;
        vhandler->set_camera_perspective(vhandler, 60);
    }
}

static void on_orthographic_item(GtkMenuItem *mi, void *user)
{
    ViewerAuxData *aux = (ViewerAuxData*)user;
    BotViewHandler *vhandler = aux->viewer->view_handler;
    if(vhandler) {
        aux->active_camtrans = NULL;
        vhandler->set_camera_orthographic(vhandler);
    }
}


static void
add_view_handlers (BotViewer *viewer, BotParam *param)
{
    ViewerAuxData * aux = get_viewer_aux_data (viewer);

    GtkWidget *view_menuitem = gtk_menu_item_new_with_mnemonic("_View");
    gtk_menu_bar_append(GTK_MENU_BAR(viewer->menu_bar), view_menuitem);

    GtkWidget *view_menu = gtk_menu_new();
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(view_menuitem), view_menu);

    GSList *view_list = NULL;
    GtkWidget *perspective_item = gtk_radio_menu_item_new_with_label(view_list,
                                                                     "Perspective");
    view_list =
        gtk_radio_menu_item_get_group(GTK_RADIO_MENU_ITEM(perspective_item));
    gtk_menu_append(GTK_MENU(view_menu), perspective_item);
    g_signal_connect(G_OBJECT(perspective_item), "activate",
                     G_CALLBACK(on_perspective_item), aux);

    GtkWidget *orthographic_item =
        gtk_radio_menu_item_new_with_label(view_list, "Orthographic");
    view_list =
        gtk_radio_menu_item_get_group(GTK_RADIO_MENU_ITEM(orthographic_item));
    gtk_menu_append(GTK_MENU(view_menu), orthographic_item);
    g_signal_connect(G_OBJECT(orthographic_item), "activate",
                     G_CALLBACK(on_orthographic_item), aux);
    // add camera view handlers
    aux->cam_names = bot_param_get_all_camera_names (param);
    for (int i=0; aux->cam_names && aux->cam_names[i]; i++) {
        char *cam_name = aux->cam_names[i];
        view_list = gtk_radio_menu_item_get_group (GTK_RADIO_MENU_ITEM(perspective_item));

        global_camtrans = bot_param_get_new_camtrans (param, cam_name);
        if (!global_camtrans) {
            fprintf (stdout, "add_view_handlers: Unable to find camtrans for camera %s\n", cam_name);
            continue;
        }

        char *label = g_strdup_printf ("Camera POV: %s", cam_name);
        GtkWidget *cvh_rmi = gtk_radio_menu_item_new_with_label (view_list, label);
        free (label);
        gtk_menu_shell_append (GTK_MENU_SHELL (view_menu), cvh_rmi);
        gtk_widget_show (cvh_rmi);

        g_object_set_data(G_OBJECT (cvh_rmi), "camtrans", global_camtrans);
        g_signal_connect (G_OBJECT (cvh_rmi), "activate",
                          G_CALLBACK (on_camera_view_handler_rmi_activate), aux);
    }

    gtk_widget_show_all(view_menuitem);
}

static void destroy_view_handlers (BotViewer *viewer)
{
    bot_camtrans_destroy(global_camtrans);
    ViewerAuxData *aux = get_viewer_aux_data(viewer);
    g_strfreev(aux->cam_names);
}

static void
usage (int argc, char ** argv)
{
    fprintf (stderr, "Usage: %s [OPTIONS]\n"
             "Husky visualization tool...\n"
             "\n"
             "Options:\n"
             "-p, --port <port> Set log port. 53261 is default. if 0 then no log port\n"
             "-s, --simulation  Run in simulation mode\n"
             "-h, --help        Print this help and exit\n"
             , argv[0]);
}


int main(int argc, char *argv[])
{
    gtk_init (&argc, &argv);
    glutInit (&argc, argv);
    g_thread_init (NULL);

    setlinebuf (stdout);

    struct option long_opts[] = {
        { "help",          no_argument,       NULL, 'h' },
        { "port",          required_argument, NULL, 'p' },
        { "simulation",    no_argument,       NULL, 's' },
        { 0, 0, 0, 0 }};

    int log_port=53261;
    int simulation_flag=0;
    int c;
    while ((c = getopt_long (argc, argv, "hp:s", long_opts, NULL)) >= 0) {
        switch (c) {
            case 's':
                simulation_flag = 1;
                break;
            case 'p':
                log_port = atoi(optarg);
                break;
        case 'h':
            default:
                usage (argc, argv);
                return 1;
        }
    }

    lcm_t * lcm;
    if (!(lcm = bot_lcm_get_global(NULL))) {
        fprintf (stderr, "Unable to get LCM\n");
        return 0;
    }

    BotParam * param;
    if (!(param = bot_param_get_global(lcm, 0))) {
        fprintf(stderr,"No server found : Reading from file\n");
        char config_path[2048];
        sprintf(config_path, "%s/husky.cfg", getConfigPath());
        param = bot_param_new_from_file(config_path);

        if(!param){
            fprintf (stderr, "Unable to get BotParam instance\n");
            return 0;
        }
    }

    BotFrames * frames;
    if (!(frames = bot_frames_get_global (lcm, param))) {
        fprintf (stderr, "Unable to get BotFrames instance\n");
        return 0;
    }

    bot_glib_mainloop_attach_lcm (lcm);


    BotViewer *viewer = bot_viewer_new("Husky Viewer");


    //die cleanly for control-c etc :-)
    bot_gtk_quit_on_interrupt();


    // Taken out since grabbers handle top-down view now.
    //gtk_widget_set_size_request(GTK_WIDGET(viewer->gl_area), 650, 366); // default n810 canvas size.

    // Setup auxiliary data, which is useful for some renderers
    ViewerAuxData aux_data;
    memset (&aux_data, 0, sizeof(aux_data));
    aux_data.viewer = viewer;
    aux_data.lcm = bot_lcm_get_global (NULL);
    aux_data.simulation_flag = simulation_flag;
    aux_data.frames = bot_frames_get_global (lcm, param);
    bot_core_pose_t_subscribe (aux_data.lcm, "POSE", on_pose, &aux_data);

    if (log_port>0)
        aux_data.logplayer_udp_port = log_port;

    g_object_set_data (G_OBJECT(viewer), "viewer:aux-data", &aux_data);


    // setup renderers
    setup_renderer_occupancy_map (viewer, 1, param);
    bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
    laser_util_add_renderer_to_viewer(viewer, 1, lcm, param, frames);

    bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
    bot_frames_add_renderer_to_viewer(viewer, 1, frames);
    quad_waypoint_add_renderer_to_viewer(viewer, 1, lcm);
    localize_add_renderer_to_viewer(viewer, 1, lcm);
    //log_annotation_add_renderer_to_viewer(viewer, 1, lcm, param);
    add_husky_model_renderer_to_viewer(viewer, 1, param, frames);
    //verify_check_gridmap_add_renderer_to_viewer(viewer, 1, lcm);
    navigator_plan_renderer_to_viewer(viewer, 1, lcm, frames);
    //setup_renderer_place_classification(viewer, 1, lcm, param);
    setup_renderer_rrtstar(viewer, 1, lcm);
    setup_renderer_tracks(viewer, 1, lcm, param);
    setup_renderer_simobs(viewer, 1, lcm, param);
    //setup_renderer_satellite(viewer, 1, param, frames);
    setup_renderer_gridmap(viewer, 1, lcm, param);
    setup_renderer_host_status (viewer, 1);
    renderer_sensor_status_new (viewer);
    setup_renderer_robot_status (viewer, param, 1);
    //setup_renderer_robot_commands (viewer, 1);
    occ_map_pixel_map_add_renderer_to_viewer(viewer, 1, "PIXEL_MAP", "PixelMap Viewer");
    //occ_map_voxel_map_add_renderer_to_viewer(viewer, 1, "VOXEL_MAP");
    //kinect_add_renderer_to_viewer(viewer, 0, lcm, frames, "KINECT", param);
    //setup_renderer_pcl(viewer,1,param);
    //setup_renderer_topological_graph(viewer,1,param);
    //setup_renderer_manual_calib (viewer, 1);
    //add_person_model_renderer_to_viewer(viewer, 1, param, frames, lcm);
    add_cam_thumb_renderer_to_viewer(viewer, 1, lcm, param, frames);
    //setup_renderer_vision_lcmgl (viewer, 1, lcm, param);
    setup_renderer_velodyne(viewer, 0, param, lcm);
    BotEventHandler *rs_ehandler = (BotEventHandler*) calloc(1, sizeof(BotEventHandler));
    rs_ehandler->name = "Robot State";
    rs_ehandler->enabled = 1;
    rs_ehandler->key_press = robot_state_on_key_press;
    bot_viewer_add_event_handler(viewer, rs_ehandler, 0);

    BotEventHandler *ehandler = (BotEventHandler*) calloc(1, sizeof(BotEventHandler));
    ehandler->name = "LogPlayer Remote";
    ehandler->enabled = 1;
    ehandler->key_press = logplayer_remote_on_key_press;
    bot_viewer_add_event_handler(viewer, ehandler, 0);

    add_view_handlers (viewer, param);

    char *fname = g_build_filename (g_get_user_config_dir(), ".envoy-viewerrc",
                                    NULL);
    bot_viewer_load_preferences (viewer, fname);

    gtk_main ();

    bot_viewer_save_preferences (viewer, fname);

    destroy_view_handlers (viewer);
    free (fname);
    bot_viewer_unref (viewer);
}
