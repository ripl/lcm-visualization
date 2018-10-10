/*
 * renders a OccupancyMap
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gtk/gtk.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <interfaces/map3d_interface.h>

#include <lcmtypes/map_lcmtypes.h>
#include <lcmtypes/gridmap_lcmtypes.h>

#define PARAM_SHOW_GMAPPER_MAP "Show Gmapper Map"
#define PARAM_MAP_MODE "Map Mode"
#define PARAM_DOWNSAMPLE "DownSample Large Maps"
#define PARAM_FLOOR_NO "Current Floor"
#define PARAM_NAME_OPACITY "Opacity"

#define RENDERER_NAME "OccupancyMap"

typedef struct _RendererOccupancyMap RendererOccupancyMap;

typedef enum _map_mode_t {
    GMAPPER_MAP, FRONTIER_UTILITY, CAM_FRONTIER_UTILITY, NAVIGATOR_UTILITY, NAVIGATOR_COST, MULTI_FLOOR, ALL_FLOORS
} map_mode_t;

struct _RendererOccupancyMap {
    BotRenderer renderer;
    BotEventHandler ehandler;
    lcm_t *lcm;
    BotGtkParamWidget *pw;
    gboolean param_draw_gmapper_map;
    map_mode_t param_map_mode;
    int downsample;
    GtkWidget *label;
    BotViewer *viewer;
    BotFrames *frames;

    int param_current_floor;
    int no_floors;
    int current_floor;
    int *floor_map; //keeps the floor no's of all maps
    ripl_map_t* multi_map;
    ripl_map_t map;
    ripl_map_t frontier_utility_map;
    ripl_map_t cam_frontier_utility_map;
    ripl_map_t nav_utility_map;
    ripl_map_t nav_cost_map;
    ripl_map_t * draw_map; //pointer to the map that should be being drawn
    BotGlTexture *map2dtexture;
    int texture_count;
    BotGlTexture **map2dtexture_all;
    maplcm_tagged_node_list_t *places;
};

void draw_place(float x,float y,float radius)
{
    glColor4f(1,0,0,.6);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y);
    for (int i=0; i < 360; i++){
        float degInRad = i*M_PI/180;
        glVertex2f(x+sin(degInRad)*radius,y+cos(degInRad)*radius);
    }
    glEnd();
}

void request_occupancy_map(lcm_t *lcm)
{
    /* subscripe to map, and wait for it to come in... */
    maplcm_map_request_msg_t msg;
    msg.utime =  bot_timestamp_now();
    msg.floor_no = -1;
    msg.requesting_prog = "VIEWER";

    maplcm_map_request_msg_t_publish(lcm, "MAP_REQUEST",&msg);
    //ask also for the multi-floor map
    maplcm_map_request_msg_t_publish(lcm, "MMAP_REQUEST",&msg);
}

static void map3d_place_handler(const lcm_recv_buf_t *rbuf, const char *channel, const maplcm_tagged_node_list_t *msg, void *user)
{
    RendererOccupancyMap *self = (RendererOccupancyMap*) user;
    if(self->places !=NULL){
        maplcm_tagged_node_list_t_destroy(self->places);
    }
    self->places = maplcm_tagged_node_list_t_copy(msg);
    bot_viewer_request_redraw(self->viewer);
}

static void upload_map_texture(RendererOccupancyMap *self);


static void on_obstacle_map(const lcm_recv_buf_t *rbuf, const char *channel,
                            const gmlcm_gridmap_tile_t *msg, void *user)
{
    RendererOccupancyMap *self = (RendererOccupancyMap *) user;

    bot_viewer_request_redraw(self->viewer);
}

static void gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                            const gmlcm_gridmap_t *msg, void *user)
{
    static gmlcm_gridmap_t* staticmsg = NULL;
    if (staticmsg != NULL) {
        gmlcm_gridmap_t_destroy(staticmsg);
    }

    staticmsg = gmlcm_gridmap_t_copy(msg);

    RendererOccupancyMap *self = (RendererOccupancyMap*) user;
    ripl_map_t *map = NULL;

    if ((strcmp(channel, "GMAPPER_GRIDMAP") == 0) ||
        (strcmp(channel, "MAP_SERVER")==0)) {
        fprintf(stdout,"New map received\n");
        map = &self->map;
    }
    else if (strcmp(channel, "FRONTIER_UTILITY_MAP") == 0) {
        map = &self->frontier_utility_map;
    }
    else if (strcmp(channel, "CAM_FRONTIER_UTILITY_MAP") == 0) {
        map = &self->cam_frontier_utility_map;
    }
    else if (strcmp(channel, "NAVIGATOR_UTILITY_MAP") == 0) {
        map = &self->nav_utility_map;
    }
     else if (strcmp(channel, "NAVIGATOR_COST_MAP") == 0) {
        map = &self->nav_cost_map;
    }
    else{
        //fprintf(stderr,"ERROR: UNHANDLED MAP channel\n");
        exit(1);
    }

    if (map->map != NULL)
        free(map->map);
    if (map->complete_map != NULL)
        free(map->complete_map);

    carmen3d_map_uncompress_lcm_map(map, staticmsg);

    //invert the map values
    for (int i = 0; i < map->config.x_size; i++) {
        for (int j = 0; j < map->config.y_size; j++) {
            map->map[i][j] = 1 - map->map[i][j];
            //shift the unexplored
            if (fabs(map->map[i][j] - .5) < .1)
                map->map[i][j] = .85;
        }
    }
    upload_map_texture(self);
    bot_viewer_request_redraw(self->viewer);
}

static void multi_gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, const gmlcm_multi_gridmap_t *msg, void *user)
{
    static gmlcm_multi_gridmap_t* staticmsg = NULL;
    if (staticmsg != NULL) {
        gmlcm_multi_gridmap_t_destroy(staticmsg);
    }
    staticmsg = gmlcm_multi_gridmap_t_copy(msg);

    //fprintf(stderr,"=MultiFloor map received: No Floors : %d, Current Floor_ind: %d, No: %d=\n",
    //	  staticmsg->no_floors, staticmsg->current_floor_ind,
    //  staticmsg->maps[staticmsg->current_floor_ind].floor_no);

    RendererOccupancyMap *self = (RendererOccupancyMap*) user;
    ripl_map_t *map;

    self->current_floor = staticmsg->current_floor_ind;


    for(int i=0; i< self->no_floors; i++){

        if(self->multi_map[i].map !=NULL){
            free(self->multi_map[i].map);
        }
        if(self->multi_map[i].complete_map !=NULL){
            free(self->multi_map[i].complete_map);
        }
    }

    if(self->no_floors != staticmsg->no_floors){
        self->no_floors = staticmsg->no_floors;
        self->multi_map = (ripl_map_t *) realloc(self->multi_map,
                                                  self->no_floors * sizeof(ripl_map_t));

        // Update the gtk widget to respect the new floors
        if (staticmsg->no_floors > 0)
            bot_gtk_param_widget_clear_enum (self->pw, PARAM_FLOOR_NO);

        char name[26];
        for (int i=0; i < staticmsg->no_floors; i++) {
            sprintf (name, "%d", staticmsg->maps[i].floor_no);

            bot_gtk_param_widget_modify_enum (self->pw, PARAM_FLOOR_NO, name, staticmsg->maps[i].floor_no);
        }
        bot_gtk_param_widget_set_enum (self->pw, PARAM_FLOOR_NO, staticmsg->current_floor_ind);
    }
    self->floor_map = (int *) realloc( self->floor_map, self->no_floors *sizeof(int));

    for(int m=0; m < self->no_floors; m++){
        carmen3d_map_uncompress_lcm_map(&self->multi_map[m], &staticmsg->maps[m].gridmap);
        self->floor_map[m] = staticmsg->maps[m].floor_no;

        for (int i = 0; i < self->multi_map[m].config.x_size; i++) {
            //fprintf(stderr,"Inverting\n");
            for (int j = 0; j < self->multi_map[m].config.y_size; j++) {
                self->multi_map[m].map[i][j] = 1 - self->multi_map[m].map[i][j];
                //shift the unexplored
                if (fabs(self->multi_map[m].map[i][j] - .5) < .1)
                    self->multi_map[m].map[i][j] = .85;
            }
        }
    }

    //add the current floor map to this
    map = &self->map;
    if (map->map != NULL)
        free(map->map);
    if (map->complete_map != NULL)
        free(map->complete_map);

    fprintf(stderr,"Current Floor : %d\n",self->current_floor);
    if(self->current_floor >=0){
        carmen3d_map_uncompress_lcm_map(map, &staticmsg->maps[self->current_floor].gridmap);
    }
    else{//if we do not have the current floor -
        //we draw first map
        carmen3d_map_uncompress_lcm_map(map, &staticmsg->maps[0].gridmap);
    }

    /*ripl_map_uncompress_lcm_map(map, staticmsg);

    //invert the map values
    for (int i = 0; i < map->config.x_size; i++) {
    for (int j = 0; j < map->config.y_size; j++) {
    map->map[i][j] = 1 - map->map[i][j];
    //shift the unexplored
    if (fabs(map->map[i][j] - .5) < .1)
    map->map[i][j] = .85;
    }
    }
    upload_map_texture(self);
    bot_viewer_request_redraw(self->viewer);*/

    for (int i = 0; i < map->config.x_size; i++) {
        for (int j = 0; j < map->config.y_size; j++) {
            map->map[i][j] = 1 - map->map[i][j];
            //shift the unexplored
            if (fabs(map->map[i][j] - .5) < .1)
                map->map[i][j] = .85;
        }
    }

    upload_map_texture(self);
    //fprintf(stderr,"Acquired map\n");
    //bot_viewer_request_redraw(self->viewer);
    bot_viewer_request_redraw(self->viewer);
}




static void upload_map_texture(RendererOccupancyMap *self)
{
    int multi_map_ind = -1;
    //fprintf(stderr,"Uploading texture\n");
    int draw_all_floors = 0;

    switch (self->param_map_mode) {
        case GMAPPER_MAP:
            self->draw_map = &self->map;
            break;
        case MULTI_FLOOR:
            for(int m=0; m< self->no_floors; m++){
                //fprintf(stderr,"Ind : %d => Floor No : %d\n", m , self->floor_map[m]);
                if(self->floor_map[m] != (self->param_current_floor)){
                    continue;
                }
                else{
                    fprintf(stderr,"Drawing Floor : %d Ind : %d\n", self->param_current_floor, m);
                    //fprintf(stderr,"Drawing Floor : %d\n", m);
                    self->draw_map = &self->multi_map[m];
                    multi_map_ind = m;
                    //invert

                    break;
                }
            }
            break;
        case ALL_FLOORS : // CUrrently not supported as it leaks memory
            draw_all_floors = 1;
            //fprintf(stderr,"Multi-Floor Drawing - Target Floor : %d\n", self->param_current_floor);
            //self->draw_map = &self->map;
            for(int m=0; m< self->no_floors; m++){
                //fprintf(stderr,"Ind : %d => Floor No : %d\n", m , self->floor_map[m]);
                if(self->floor_map[m] != (self->param_current_floor)){
                    continue;
                }
                else{
                    fprintf(stderr,"Drawing Floor : %d Ind : %d\n", self->param_current_floor, m);
                    //fprintf(stderr,"Drawing Floor : %d\n", m);
                    self->draw_map = &self->multi_map[m];
                    multi_map_ind = m;
                    //invert

                    break;
                }
            }
            break;
        case FRONTIER_UTILITY:
            self->draw_map = &self->frontier_utility_map;
            break;
        case CAM_FRONTIER_UTILITY:
            self->draw_map = &self->cam_frontier_utility_map;
            break;
        case NAVIGATOR_UTILITY:
            self->draw_map = &self->nav_utility_map;
            break;
        case NAVIGATOR_COST:
            self->draw_map = &self->nav_cost_map;
            break;
    }

    float * draw_complete_map = NULL;
    static CvMat * draw_map_im = NULL;
    if (self->draw_map->map && self->param_draw_gmapper_map) {
        if(!draw_all_floors){
            int maxDrawDim = 1024;
            static int old_x_size = 0;
            static int old_y_size = 0;
            int x_size = 0;
            int y_size = 0;
            if (self->downsample && (self->draw_map->config.x_size > maxDrawDim || self->draw_map->config.y_size > maxDrawDim)) {
                //HUGE map... need to downsample
                double AR = (double) self->draw_map->config.y_size / (double) self->draw_map->config.x_size;
                if (AR <= 1) {
                    x_size = maxDrawDim;
                    y_size = (int) (((double) maxDrawDim) * AR);
                }
                else {
                    x_size = (int) (((double) maxDrawDim) * 1 / AR);
                    y_size = maxDrawDim;
                }
                //      fprintf(stderr,"map is (%d,%d), downsampling to (%d,%d)\n",self->draw_map->config.x_size,self->draw_map->config.y_size,x_size,y_size);
                if (draw_map_im == NULL) {
                    draw_map_im = cvCreateMat(x_size, y_size, CV_32FC1);
                }
                else {
                    CvSize s = cvGetSize(draw_map_im);
                    if (s.height != x_size || s.width != y_size) {
                        //need to resize matrix
                        cvReleaseMat(&draw_map_im);
                        draw_map_im = cvCreateMat(x_size, y_size, CV_32FC1);
                    }
                }

                CvMat map_im = cvMat(self->draw_map->config.x_size, self->draw_map->config.y_size, CV_32FC1,
                                     self->draw_map->complete_map);
                cvResize(&map_im, draw_map_im, CV_INTER_NN);
                draw_complete_map = (float*) cvPtr2D(draw_map_im, 0, 0, NULL);
            }
            else {
                x_size = self->draw_map->config.x_size;
                y_size = self->draw_map->config.y_size;
                draw_complete_map = self->draw_map->complete_map;

            }

            // create the texture object if necessary
            if (self->map2dtexture == NULL || (x_size != old_x_size || y_size != old_y_size)) {
                if (self->map2dtexture != NULL)
                    bot_gl_texture_free(self->map2dtexture);
                int data_size = x_size * y_size * sizeof(float);
                self->map2dtexture = bot_gl_texture_new(y_size, x_size, data_size);
                old_x_size = x_size;
                old_y_size = y_size;
            }

            bot_gl_texture_upload(self->map2dtexture, GL_LUMINANCE, GL_FLOAT, y_size * sizeof(float), draw_complete_map);

            //draw places
            //for some reaason not being drawn properly
            /*if(self->places !=NULL){
              fprintf(stderr,"Have tagged Nodes\n");
              if(self->param_map_mode == MULTI_FLOOR){
              int i;
              double textpos[3];
              for(i=0;i< self->places->place_count;i++){
              //if(self->places->places[i].floor_index != multi_map_ind)
              //  continue;

              double global_pl_x, global_pl_y;

              global_pl_x = self->places->places[i].x;// -  self->draw_map->midpt.x + self->draw_map->map_zero.x;
              global_pl_y = self->places->places[i].y;// - self->draw_map->midpt.y + self->draw_map->map_zero.y;


              draw_place(global_pl_x, global_pl_y,0.2);
              fprintf(stderr,"Drawing (%f,%f)\n", global_pl_x, global_pl_y);
              glColor3f(1, 1, 1);

              textpos[0] = global_pl_x + 1.0;
              textpos[1] = global_pl_y + 1.0;
              textpos[2] = 2;
              bot_gl_draw_text(textpos, GLUT_BITMAP_HELVETICA_12, self->places->places[i].label, BOT_GL_DRAW_TEXT_DROP_SHADOW);
              }
              }
              }*/
            //else use the current floor

        }
        else{
            //add textures for all the floors
            if(self->map2dtexture_all){
                for(int i=0;i < self->texture_count; i++){
                    if (self->map2dtexture_all[i] != NULL)
                        bot_gl_texture_free(self->map2dtexture_all[i]);
                }
                free(self->map2dtexture_all);
            }
            self->texture_count = self->no_floors;
            self->map2dtexture_all = (BotGlTexture **) calloc(self->texture_count, sizeof(BotGlTexture *));
            for(int i=0;i < self->texture_count; i++){
                self->map2dtexture_all[i] = NULL;
            }

            for(int m=0; m< self->no_floors; m++){
                self->draw_map = &self->multi_map[m];
                multi_map_ind = m;

                fprintf(stderr,"Drawing Floor Index : %d\n", m);

                int maxDrawDim = 1024;
                static int old_x_size = 0;
                static int old_y_size = 0;
                int x_size = 0;
                int y_size = 0;
                if (0 && self->downsample && (self->draw_map->config.x_size > maxDrawDim || self->draw_map->config.y_size > maxDrawDim)) {
                    //HUGE map... need to downsample

                    //*** this will add a memory leak
                    if(draw_complete_map){ //this is also kind of wrong
                        free(draw_complete_map);
                    }

                    double AR = (double) self->draw_map->config.y_size / (double) self->draw_map->config.x_size;
                    if (AR <= 1) {
                        x_size = maxDrawDim;
                        y_size = (int) (((double) maxDrawDim) * AR);
                    }
                    else {
                        x_size = (int) (((double) maxDrawDim) * 1 / AR);
                        y_size = maxDrawDim;
                    }
                    //      fprintf(stderr,"map is (%d,%d), downsampling to (%d,%d)\n",self->draw_map->config.x_size,self->draw_map->config.y_size,x_size,y_size);
                    if (draw_map_im == NULL) {
                        draw_map_im = cvCreateMat(x_size, y_size, CV_32FC1);
                    }
                    else {
                        CvSize s = cvGetSize(draw_map_im);
                        if (s.height != x_size || s.width != y_size) {
                            //need to resize matrix
                            cvReleaseMat(&draw_map_im);
                            draw_map_im = cvCreateMat(x_size, y_size, CV_32FC1);
                        }
                    }

                    CvMat map_im = cvMat(self->draw_map->config.x_size, self->draw_map->config.y_size, CV_32FC1,
                                         self->draw_map->complete_map);
                    cvResize(&map_im, draw_map_im, CV_INTER_NN);
                    draw_complete_map = (float*) cvPtr2D(draw_map_im, 0, 0, NULL);
                }
                else {
                    x_size = self->draw_map->config.x_size;
                    y_size = self->draw_map->config.y_size;

                    draw_complete_map = self->draw_map->complete_map;
                }

                // create the texture object if necessary
                if (self->map2dtexture_all[m] == NULL || (x_size != old_x_size || y_size != old_y_size)) {
                    if (self->map2dtexture_all[m] != NULL)
                        bot_gl_texture_free(self->map2dtexture_all[m]);
                    int data_size = x_size * y_size * sizeof(float);
                    self->map2dtexture_all[m] = bot_gl_texture_new(y_size, x_size, data_size);
                    old_x_size = x_size;
                    old_y_size = y_size;
                }

                bot_gl_texture_upload(self->map2dtexture_all[m], GL_LUMINANCE, GL_FLOAT, y_size * sizeof(float), draw_complete_map);
            }
        }
    }
    else {
        if (self->map2dtexture != NULL) {
            bot_gl_texture_free(self->map2dtexture);
            self->map2dtexture = NULL;
        }
    }

    /*if(draw_complete_map != NULL){
        free(draw_complete_map);
        }*/

}

static void OccupancyMap_draw(BotViewer *viewer, BotRenderer *renderer)
{
    RendererOccupancyMap *self = (RendererOccupancyMap*) renderer;

    glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
    glEnable (GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable (GL_COLOR_MATERIAL);

    int draw_all_floors = 0;

    switch (self->param_map_mode) {
        case ALL_FLOORS:
            draw_all_floors = 1;
    }

    if (self->map2dtexture && self->param_draw_gmapper_map && !draw_all_floors) {
        glColor4d(1.0, 1.0, 1.0,
                  bot_gtk_param_widget_get_double (self->pw, PARAM_NAME_OPACITY));

        double global_tl[3], global_br[3];
        global_tl[2] = 0;
        global_br[2] = 0;
        carmen3d_map3d_map_index_to_global_coordinates(&global_tl[0], &global_tl[1], self->draw_map->midpt,
                                                       self->draw_map->map_zero, self->draw_map->config.resolution, 0, 0);
        carmen3d_map3d_map_index_to_global_coordinates(&global_br[0], &global_br[1], self->draw_map->midpt,
                                                       self->draw_map->map_zero, self->draw_map->config.resolution, self->draw_map->config.x_size,
                                                       self->draw_map->config.y_size);
        //bot_gl_texture_draw_coords(self->map2dtexture,
        //                           global_tl[0], global_tl[1], 0,
        //                           global_br[0], global_tl[1], 0,
        //                           global_br[0], global_br[1], 0,
        //                           global_tl[0], global_br[1], 0);

        double global_bl[] = {global_br[0], global_tl[1], 0};
        double global_tr[] = {global_tl[0], global_br[1], 0};


        // Transform the global coordinates into the local frame
        double local_tl[3], local_bl[3], local_tr[3], local_br[3];
        bot_frames_transform_vec (self->frames, "global", "local", global_tl, local_tl);
        bot_frames_transform_vec (self->frames, "global", "local", global_bl, local_bl);
        bot_frames_transform_vec (self->frames, "global", "local", global_tr, local_tr);
        bot_frames_transform_vec (self->frames, "global", "local", global_br, local_br);

        // Draw at a negative z to avoid occluding
        double z = -0.01;
        bot_gl_texture_draw_coords(self->map2dtexture,
                                   local_tl[0], local_tl[1], z,
                                   local_bl[0], local_bl[1], z,
                                   local_br[0], local_br[1], z,
                                   local_tr[0], local_tr[1], z);
    }

    if (self->map2dtexture_all && self->param_draw_gmapper_map && draw_all_floors) {
        //fprintf(stderr,"Drawing All floors\n");
        for(int i=0;i < self->texture_count; i++){
            float depth = (self->floor_map[i] - (self->param_current_floor))* 10.0;

            //fprintf(stderr,"Depth : %f [%d] = %d - %d\n", depth, i, self->floor_map[i], self->param_current_floor);

            glPushMatrix();
            glTranslated(0, 0, depth);
            //make the current floor 0 - and the rest + or - based on the floor no

            glColor4d(1.0, 1.0, 1.0,
                      bot_gtk_param_widget_get_double (self->pw, PARAM_NAME_OPACITY));

            self->draw_map = &self->multi_map[i];

            double global_tl[3], global_br[3];
            global_tl[2] = 0;
            global_br[2] = 0;
            carmen3d_map3d_map_index_to_global_coordinates(&global_tl[0], &global_tl[1], self->draw_map->midpt,
                                                           self->draw_map->map_zero, self->draw_map->config.resolution, 0, 0);
            carmen3d_map3d_map_index_to_global_coordinates(&global_br[0], &global_br[1], self->draw_map->midpt,
                                                           self->draw_map->map_zero, self->draw_map->config.resolution, self->draw_map->config.x_size,
                                                           self->draw_map->config.y_size);
            //bot_gl_texture_draw_coords(self->map2dtexture,
            //                           global_tl[0], global_tl[1], 0,
            //                           global_br[0], global_tl[1], 0,
            //                           global_br[0], global_br[1], 0,
            //                           global_tl[0], global_br[1], 0);

            double global_bl[] = {global_br[0], global_tl[1], 0};
            double global_tr[] = {global_tl[0], global_br[1], 0};


            // Transform the global coordinates into the local frame
            double local_tl[3], local_bl[3], local_tr[3], local_br[3];
            bot_frames_transform_vec (self->frames, "global", "local", global_tl, local_tl);
            bot_frames_transform_vec (self->frames, "global", "local", global_bl, local_bl);
            bot_frames_transform_vec (self->frames, "global", "local", global_tr, local_tr);
            bot_frames_transform_vec (self->frames, "global", "local", global_br, local_br);

            bot_gl_texture_draw_coords(self->map2dtexture_all[i],
                                       local_tl[0], local_tl[1], 0,
                                       local_bl[0], local_bl[1], 0,
                                       local_br[0], local_br[1], 0,
                                       local_tr[0], local_tr[1], 0);

            glPopMatrix();
        }
    }
    //glDisable (GL_COLOR_MATERIAL);
    //glDisable (GL_DEPTH_TEST);
    glPopAttrib ();
}

static void upload_map_texture_single(RendererOccupancyMap *self)
{
    int multi_map_ind = -1;
    //fprintf(stderr,"Uploading texture\n");
    switch (self->param_map_mode) {
    case GMAPPER_MAP:
        self->draw_map = &self->map;
        break;
    case MULTI_FLOOR:
        //fprintf(stderr,"Multi-Floor Drawing - Target Floor : %d\n", self->param_current_floor);
        //self->draw_map = &self->map;
        for(int m=0; m< self->no_floors; m++){
            //fprintf(stderr,"Ind : %d => Floor No : %d\n", m , self->floor_map[m]);
            if(self->floor_map[m] != (self->param_current_floor)){
                continue;
            }
            else{
                fprintf(stderr,"Drawing Floor : %d Ind : %d\n", self->param_current_floor, m);
                //fprintf(stderr,"Drawing Floor : %d\n", m);
                self->draw_map = &self->multi_map[m];
                multi_map_ind = m;
                //invert

                break;
            }
        }
        break;
    case FRONTIER_UTILITY:
        self->draw_map = &self->frontier_utility_map;
        break;
    case CAM_FRONTIER_UTILITY:
        self->draw_map = &self->cam_frontier_utility_map;
        break;
    case NAVIGATOR_UTILITY:
        self->draw_map = &self->nav_cost_map;
        break;
    }

    float * draw_complete_map = NULL;
    static CvMat * draw_map_im = NULL;
    if (self->draw_map->map && self->param_draw_gmapper_map) {
        int maxDrawDim = 1024;
        static int old_x_size = 0;
        static int old_y_size = 0;
        int x_size = 0;
        int y_size = 0;
        if (self->downsample && (self->draw_map->config.x_size > maxDrawDim || self->draw_map->config.y_size > maxDrawDim)) {
            //HUGE map... need to downsample
            double AR = (double) self->draw_map->config.y_size / (double) self->draw_map->config.x_size;
            if (AR <= 1) {
                x_size = maxDrawDim;
                y_size = (int) (((double) maxDrawDim) * AR);
            }
            else {
                x_size = (int) (((double) maxDrawDim) * 1 / AR);
                y_size = maxDrawDim;
            }
            //      fprintf(stderr,"map is (%d,%d), downsampling to (%d,%d)\n",self->draw_map->config.x_size,self->draw_map->config.y_size,x_size,y_size);
            if (draw_map_im == NULL) {
                draw_map_im = cvCreateMat(x_size, y_size, CV_32FC1);
            }
            else {
                CvSize s = cvGetSize(draw_map_im);
                if (s.height != x_size || s.width != y_size) {
                    //need to resize matrix
                    cvReleaseMat(&draw_map_im);
                    draw_map_im = cvCreateMat(x_size, y_size, CV_32FC1);
                }
            }

            CvMat map_im = cvMat(self->draw_map->config.x_size, self->draw_map->config.y_size, CV_32FC1,
                                 self->draw_map->complete_map);
            cvResize(&map_im, draw_map_im, CV_INTER_NN);
            draw_complete_map = (float*) cvPtr2D(draw_map_im, 0, 0, NULL);
        }
        else {
            x_size = self->draw_map->config.x_size;
            y_size = self->draw_map->config.y_size;
            draw_complete_map = self->draw_map->complete_map;
        }

        // create the texture object if necessary
        if (self->map2dtexture == NULL || (x_size != old_x_size || y_size != old_y_size)) {
            if (self->map2dtexture != NULL)
                bot_gl_texture_free(self->map2dtexture);
            int data_size = x_size * y_size * sizeof(float);
            self->map2dtexture = bot_gl_texture_new(y_size, x_size, data_size);
            old_x_size = x_size;
            old_y_size = y_size;
        }

        bot_gl_texture_upload(self->map2dtexture, GL_LUMINANCE, GL_FLOAT, y_size * sizeof(float), draw_complete_map);

        //draw places
        //for some reaason not being drawn properly
        /*if(self->places !=NULL){
          fprintf(stderr,"Have tagged Nodes\n");
          if(self->param_map_mode == MULTI_FLOOR){
          int i;
          double textpos[3];
          for(i=0;i< self->places->place_count;i++){
          //if(self->places->places[i].floor_index != multi_map_ind)
          //  continue;

          double global_pl_x, global_pl_y;

          global_pl_x = self->places->places[i].x;// -  self->draw_map->midpt.x + self->draw_map->map_zero.x;
          global_pl_y = self->places->places[i].y;// - self->draw_map->midpt.y + self->draw_map->map_zero.y;


          draw_place(global_pl_x, global_pl_y,0.2);
          fprintf(stderr,"Drawing (%f,%f)\n", global_pl_x, global_pl_y);
          glColor3f(1, 1, 1);

          textpos[0] = global_pl_x + 1.0;
          textpos[1] = global_pl_y + 1.0;
          textpos[2] = 2;
          bot_gl_draw_text(textpos, GLUT_BITMAP_HELVETICA_12, self->places->places[i].label, BOT_GL_DRAW_TEXT_DROP_SHADOW);
          }
          }
          }*/
        //else use the current floor

    }
    else {
        if (self->map2dtexture != NULL) {
            bot_gl_texture_free(self->map2dtexture);
            self->map2dtexture = NULL;
        }
    }

}

static void OccupancyMap_free(BotRenderer *renderer)
{
    free(renderer);
}

static int mouse_press(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3],
                       const GdkEventButton *event)
{
    //  RendererOccupancyMap *self = (RendererOccupancyMap*) ehandler->user;
    return 0;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererOccupancyMap *self = (RendererOccupancyMap*) user;

    self->param_draw_gmapper_map = bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_GMAPPER_MAP);
    self->param_map_mode = bot_gtk_param_widget_get_enum(self->pw, PARAM_MAP_MODE);
    self->downsample = bot_gtk_param_widget_get_bool(self->pw,PARAM_DOWNSAMPLE);
    self->param_current_floor = bot_gtk_param_widget_get_enum(self->pw, PARAM_FLOOR_NO);
    upload_map_texture(self);
    bot_viewer_request_redraw(self->viewer);
}

static BotRenderer*
renderer_occupancy_map_new(BotViewer *viewer, int render_priority, BotParam * _param)
{
    RendererOccupancyMap *self = (RendererOccupancyMap*) calloc(1, sizeof(RendererOccupancyMap));
    BotRenderer *renderer = &self->renderer;
    self->viewer = viewer;
    self->lcm = bot_lcm_get_global (NULL);
    self->map.map = NULL;
    self->map.complete_map = NULL;
    self->map2dtexture = NULL;
    self->param_map_mode = GMAPPER_MAP;

    BotParam *param = _param;
    self->frames = bot_frames_get_global (self->lcm, param);

    renderer->draw = OccupancyMap_draw;
    renderer->destroy = OccupancyMap_free;
    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->key_press = NULL;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    // --- SETUP SIDE BOX WIDGET
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_SHOW_GMAPPER_MAP, 1, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_DOWNSAMPLE, 1, NULL);
    bot_gtk_param_widget_add_double (self->pw, PARAM_NAME_OPACITY,
                                     BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.05, 0.5);


    self->param_draw_gmapper_map = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_GMAPPER_MAP);
    bot_gtk_param_widget_add_enum(self->pw, PARAM_MAP_MODE, BOT_GTK_PARAM_WIDGET_MENU, self->param_map_mode, "GMapping",
                                  //GMAPPER_MAP, "Multi Floor", MULTI_FLOOR, "All Floors", ALL_FLOORS, "Navigator Utility", NAVIGATOR_UTILITY, // ALL FLOORS currently leaks memory
                                  GMAPPER_MAP, "Multi Floor", MULTI_FLOOR, "Navigator Utility", NAVIGATOR_UTILITY,
                                  "Navigator Cost", NAVIGATOR_COST, "Frontier Utility", FRONTIER_UTILITY, "CAM Utility",
                                  CAM_FRONTIER_UTILITY, NULL);

    /* bot_gtk_param_widget_add_enum(self->pw, PARAM_FLOOR_NO, BOT_GTK_PARAM_WIDGET_MENU,  */
    /*                               self->param_current_floor,  */
    /*                               "Ground",0, */
    /*                               "First",1, */
    /*                               "Second", 2,  */
    /*                               "Third",3, */
    /*                               "Fourth", 4,  */
    /*                               "Fifth", 5, NULL); */

    bot_gtk_param_widget_add_enum(self->pw, PARAM_FLOOR_NO, BOT_GTK_PARAM_WIDGET_MENU,
                                  self->param_current_floor,
                                  "Default",0, NULL);

    gtk_widget_show_all(renderer->widget);
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    on_param_widget_changed(self->pw, "", self);

    gmlcm_gridmap_t_subscribe(self->lcm, "GRIDMAP", gridmap_handler, self);
    gmlcm_gridmap_t_subscribe(self->lcm, "MAP_SERVER", gridmap_handler, self);
    gmlcm_multi_gridmap_t_subscribe(self->lcm, "MULTI_FLOOR_MAPS", multi_gridmap_handler, self);
    gmlcm_multi_gridmap_t_subscribe(self->lcm, "MMAP_SERVER", multi_gridmap_handler, self);
    gmlcm_gridmap_t_subscribe(self->lcm, "FRONTIER_UTILITY_MAP", gridmap_handler, self);
    gmlcm_gridmap_t_subscribe(self->lcm, "NAVIGATOR_UTILITY_MAP", gridmap_handler, self);
    gmlcm_gridmap_t_subscribe(self->lcm, "NAVIGATOR_COST_MAP", gridmap_handler, self);
    gmlcm_gridmap_t_subscribe(self->lcm, "CAM_FRONTIER_UTILITY_MAP", gridmap_handler, self);
    maplcm_tagged_node_list_t_subscribe(self->lcm, "TAGGED_NODES", map3d_place_handler, self);

    // Subscribe to the obstacle map for the sake of re-rendering the occupancy map with the most recent global-to-local
    gmlcm_gridmap_tile_t_subscribe (self->lcm, "OBSTACLE_MAP", on_obstacle_map, self);

    request_occupancy_map(self->lcm);
    return &self->renderer;
}

void setup_renderer_occupancy_map (BotViewer *viewer, int priority, BotParam * param)
{
    BotRenderer* renderer = renderer_occupancy_map_new(viewer, priority, param);
    bot_viewer_add_renderer(viewer, renderer, priority);
}
