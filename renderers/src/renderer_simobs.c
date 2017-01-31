#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <gdk/gdkkeysyms.h>


#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/gtk_util.h>
#include <bot_vis/viewer.h>
#include <bot_param/param_client.h>

#include <bot_core/math_util.h>

#include <geom_utils/geometry.h>

#include <pthread.h>

//#include <lcmtypes/erlcm_rect_list_t.h>
#include <lcmtypes/hr_lcmtypes.h>

#include "viewer_aux_data.h"

#define RENDERER_NAME "SimObstacle"

#define TRANSMIT_OBSTACLES_HZ 10
#define MIN_SIZE 0.3

#define PARAM_PEPPER "Pepper" 
#define NUM_PEPPER_GRAINS 8 

#define SQ(x) ((x)*(x))
#define TO_DEG(x) ((x)*180.0/M_PI)

typedef struct _RendererSimObs {
    BotRenderer     renderer;
    BotEventHandler ehandler;
    BotViewer       *viewer;
    lcm_t           *lcm;
    BotFrames       *frames;
    BotGtkParamWidget *pw;
 
    GArray        *rects;
    double         rects_xy[2];
    pthread_mutex_t mutex;

    int current_floor_ind;
    int current_floor_no;

    // this is the rect that is being edited (depending
    // on the state)
    int edit_idx;

    // last rect that we found in a query
    int hover_idx;
    double lastxy[2];

    // set to one if the user manually gave us "pick" by pressing a key
    int         key_pick;
    int transmitter;
} RendererSimObs;

void activate_simobs_transmitter(RendererSimObs *self);


void get_current_floor(RendererSimObs *s){
    //send a floor status update message                                
    erlcm_map_request_msg_t msg;
    msg.utime = bot_timestamp_now(); 
    msg.requesting_prog = "NAV_RENDERER";    
    erlcm_map_request_msg_t_publish(s->lcm, "FLOOR_STATUS_REQUEST", &msg);  
}



static void
on_sim_rects_new(const lcm_recv_buf_t *rbuf, const char *chennel, 
                 const erlcm_rect_list_t *msg, void *_user)
{
    RendererSimObs *self = (RendererSimObs*) _user;
 
    pthread_mutex_lock(&self->mutex);

    // clear our state first
    self->ehandler.picking = 0;
    self->edit_idx = -1;
    self->hover_idx = -1;
    g_array_set_size(self->rects, msg->num_rects);

    // set up.
    self->rects_xy[0] = msg->xy[0];
    self->rects_xy[1] = msg->xy[1];
    memcpy(self->rects->data,msg->rects,sizeof(erlcm_rect_t)*self->rects->len);

    if (!self->transmitter) {
        activate_simobs_transmitter(self);
    }
 
    pthread_mutex_unlock(&self->mutex);
}



static int find_idx(RendererSimObs *self, const double xy[2])
{
    double best_distance = HUGE;
    int best_idx = -1;

    for (unsigned int i = 0; i < self->rects->len; i++) {
        erlcm_rect_t *rect = &g_array_index(self->rects, erlcm_rect_t, i);
        double dx = xy[0] - (self->rects_xy[0] + rect->dxy[0]);
        double dy = xy[1] - (self->rects_xy[1] + rect->dxy[1]);
        double distance = sqrt(SQ(dx) + SQ(dy));
        double c,s;
        bot_fasttrig_sincos(-rect->theta,&s,&c);
        double tx = c*dx - s*dy;
        double ty = s*dx + c*dy;

        if (distance < best_distance && fabs(tx) < rect->size[0]/2 && fabs(ty) < rect->size[1]/2) {
            best_distance = distance;
            best_idx = i;
        }
    }

    return best_idx;
}

static void floor_status_handler(const lcm_recv_buf_t *rbuf, const char *channel, 
                                 const erlcm_floor_status_msg_t *msg, void *user)
{
    fprintf(stderr, "Floor Status Received : Ind %d No ; %d", msg->floor_ind, msg->floor_no);

    RendererSimObs *self = (RendererSimObs*) user;

    self->current_floor_ind = msg->floor_ind;
    self->current_floor_no = msg->floor_no;
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event)
{
    RendererSimObs *self = (RendererSimObs*) ehandler->user;

    if (event->keyval == 'o' || event->keyval == 'O'){
        get_current_floor(self);

        if(!self->key_pick) {
            bot_viewer_request_pick(viewer, ehandler);
            self->key_pick = 1;
            return 1;
        }
        else{
            ehandler->picking = 0;
            self->key_pick = 0;
        }
    }
    

    if (self->edit_idx>-1 && (event->keyval == GDK_Delete || 
                              event->keyval == GDK_BackSpace)) {
        
        pthread_mutex_lock(&self->mutex);
        // find rect index
        if (self->edit_idx>-1)
            g_array_remove_index_fast(self->rects, self->edit_idx);
        self->edit_idx = -1;
        self->hover_idx = -1;
        pthread_mutex_unlock(&self->mutex);
        bot_viewer_request_redraw(viewer);
        return 1;
    }


    if (event->keyval == GDK_Escape) {
        ehandler->picking = 0;
        self->key_pick = 0;
    }

    return 0;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler,
                        const double ray_start[3], const double ray_dir[3], 
                        const GdkEventButton *event)
{
    RendererSimObs *self = (RendererSimObs*) ehandler->user;

    double xy[2];
    int consumed = 0;

    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir), 
            0, POINT2D(xy));

    memcpy(self->lastxy, xy, 2 * sizeof(double));

    // is the CONTROL key depressed
    int control = event->state & GDK_CONTROL_MASK;

    // it could be a pick (which begins a move)
    // or creation of a new click (requires CNTRL key to be depressed)

    // selecting a previously-created rect
    if (event->button == 1 && ehandler->picking) {
        self->edit_idx = find_idx(self, xy);
        if (self->edit_idx >= 0)
            consumed = 1;
    } 

    // creating a new rect requires that the CTRL key be pressed
    if (ehandler->picking && self->edit_idx <0 && event->button == 1 && control) {
        // creating a new rect
        erlcm_rect_t rect;

        rect.dxy[0] = xy[0] - self->rects_xy[0];
        rect.dxy[1] = xy[1] - self->rects_xy[1];
        rect.size[0] = 1;
        rect.size[1] = 1;
        rect.theta = 0;
        rect.floor_ind = self->current_floor_ind; 
        rect.floor_no = self->current_floor_no; 
        g_array_append_val(self->rects, rect);
        self->edit_idx = self->rects->len-1;
        consumed = 1;
        bot_viewer_request_pick(viewer, ehandler);
        activate_simobs_transmitter(self);
    } 

    bot_viewer_request_redraw(viewer);

    return consumed;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], const GdkEventMotion *event)
{
    RendererSimObs *self = (RendererSimObs*) ehandler->user;
 
    double xy[2];
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir), 
            0, POINT2D(xy));

    int consumed = 0;
    int control = event->state & GDK_CONTROL_MASK;
    int shift = event->state & GDK_SHIFT_MASK;

    // only handle mouse button 1.
    if (!(event->state & GDK_BUTTON1_MASK))
        return 0;

    if (self->edit_idx<0)
        return 0;

    erlcm_rect_t *rect = &g_array_index(self->rects, erlcm_rect_t, self->edit_idx);
    double real_xy[2] = {self->rects_xy[0] + rect->dxy[0], self->rects_xy[1] + rect->dxy[1]};
    if (event->state==GDK_BUTTON1_MASK) {
      
        //  gps_linearize_to_lat_lon(self->gpslin, xy, self->edit_rect->ll);
        rect->dxy[0]= xy[0] - self->rects_xy[0];
        rect->dxy[1]= xy[1] - self->rects_xy[1];
        consumed = 1;
        
    } else if (control) { 
      
        double sx = xy[0] - real_xy[0];
        double sy = xy[1] - real_xy[1];
        
        double tx = cos(-rect->theta)*sx - sin(-rect->theta)*sy;
        double ty = sin(-rect->theta)*sx + cos(-rect->theta)*sy;
        
        rect->size[0] = fmax(fabs(2*tx), MIN_SIZE);
        rect->size[1] = fmax(fabs(2*ty), MIN_SIZE);
        consumed = 1;
        
    } else if (shift) {
        
        double dtheta = 
            atan2(xy[1] - real_xy[1],
                  xy[0] - real_xy[0]) - 
            atan2(self->lastxy[1] - real_xy[1],
                  self->lastxy[0] - real_xy[0]);
        
        memcpy(self->lastxy, xy, 2 * sizeof(double));
        
        rect->theta += dtheta;
        
        consumed = 1;
    } 
    
    bot_viewer_request_redraw(viewer);
    return consumed;
}

static int mouse_release (BotViewer *viewer, BotEventHandler *ehandler,
                          const double ray_start[3], const double ray_dir[3], 
                          const GdkEventButton *event)
{
    RendererSimObs *self = (RendererSimObs*) ehandler->user;

    self->edit_idx = -1;
    if (!self->key_pick)
        ehandler->picking = 0;
    
    bot_viewer_request_redraw(viewer);
    return 1;
}

static double pick_query(BotViewer *viewer, BotEventHandler *ehandler, 
                         const double ray_start[3], const double ray_dir[3])
{
    RendererSimObs *self = (RendererSimObs*) ehandler->user;

    double xy[2];
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
            POINT3D(ray_dir), 0, POINT2D(xy));

    self->hover_idx = find_idx(self, xy);
    if (self->hover_idx<0)
        return -1;
    
    erlcm_rect_t *rect = &g_array_index(self->rects, erlcm_rect_t, self->hover_idx);

    return sqrt(SQ(self->rects_xy[0] + rect->dxy[0] - xy[0]) + 
                SQ(self->rects_xy[0] + rect->dxy[1] - xy[1]));
}

static void draw_rect (RendererSimObs *self, erlcm_rect_t *rect, int draw_orient)
{
    //double cxy[2];
//    gps_linearize_to_xy(self->gpslin, rect->ll, cxy);

    glPushMatrix();
    double real_xy[2] = { self->rects_xy[0] + rect->dxy[0], self->rects_xy[1] + rect->dxy[1]};
 
    glTranslated(real_xy[0], real_xy[1], 0);
    glRotated(TO_DEG(rect->theta), 0, 0, 1);
    glTranslated(-real_xy[0], -real_xy[1], 0);

    glEnable(GL_BLEND);
    glBegin(GL_QUADS);
    glVertex2d(real_xy[0] - rect->size[0]/2, real_xy[1] - rect->size[1]/2);
    glVertex2d(real_xy[0] - rect->size[0]/2, real_xy[1] + rect->size[1]/2);
    glVertex2d(real_xy[0] + rect->size[0]/2, real_xy[1] + rect->size[1]/2);
    glVertex2d(real_xy[0] + rect->size[0]/2, real_xy[1] - rect->size[1]/2);
    glEnd();    
    glPopMatrix();
    
    if (draw_orient) {
        glBegin(GL_LINES);
        glColor3f(1, 1, 0);
        glVertex2d(real_xy[0], real_xy[1]);
        double dx = cos(rect->theta)*rect->size[0]/2 - sin(rect->theta)*rect->size[1]/2;
        double dy = sin(rect->theta)*rect->size[0]/2 + cos(rect->theta)*rect->size[1]/2;
        glVertex2d(real_xy[0] + dx, real_xy[1] + dy);
        glEnd();
    }
}

static void renderer_simobs_draw( BotViewer *viewer, BotRenderer *super )
{
    RendererSimObs *self = (RendererSimObs*) super->user;

    for (unsigned int i = 0; i < self->rects->len; i++) {
        erlcm_rect_t *rect = &g_array_index(self->rects, erlcm_rect_t, i);
        if(rect->floor_ind != self->current_floor_ind){
            continue;
        }
        
        if (self->edit_idx == i)
            glColor4f(1, .0, .5, .5);
        else if (self->ehandler.hovering && self->hover_idx == i)
            glColor4f(.5, 1, 1, .5);
        else
            glColor4f(1, .0, .0, 0.3);

        draw_rect(self, rect, 0);
    }
}    

gboolean on_transmit_obs(gpointer _user)
{
    RendererSimObs *self = (RendererSimObs*) _user;
    
    double pos_body[3] = {0, 0, 0};
    double pos_local[3];
    if (!bot_frames_transform_vec (self->frames, "body", "local", pos_body, pos_local))
        return TRUE;
    
    int pepper = 0;
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_PEPPER))
        pepper = NUM_PEPPER_GRAINS;

    /*erlcm_floor_rect_list_t floor_rects; 
    //    memcpy(&(floor_rects.rect_list), rects, sizeof(erlcm_rect_list_t));
    //    floor_rects.rect_list = rects; 
    floor_rects.floor_ind = self->current_floor_ind; 
    floor_rects.floor_no = self->current_floor_no; 

    memset(&(floor_rects.rect_list), 0, sizeof(erlcm_rect_list_t));
    erlcm_rect_list_t *rects = &(floor_rects.rect_list); */
    erlcm_rect_list_t *rects = (erlcm_rect_list_t*) calloc(1, (pepper?pepper:1)*sizeof(erlcm_rect_list_t));
    rects->utime = bot_timestamp_now();
    rects->num_rects = self->rects->len*(pepper?pepper:1);
    rects->rects = (erlcm_rect_t*) calloc(rects->num_rects, sizeof(erlcm_rect_t));    

    double cxyz[3] = { self->rects_xy[0], self->rects_xy[1], 0};

    rects->xy[0] = cxyz[0];
    rects->xy[1] = cxyz[1];
    int rect_idx =0;
    for (unsigned int i = 0; i < self->rects->len; i++) {
        erlcm_rect_t *myrect = & g_array_index(self->rects, erlcm_rect_t,i);
        if (pepper) {
            double pose_dir = bot_fasttrig_atan2(pos_local[1]-(cxyz[1] +myrect->dxy[1]),
                                                 pos_local[0]-(cxyz[0] +myrect->dxy[0]));
            double s,c;
            bot_fasttrig_sincos(-myrect->theta,&s,&c);
            double s2,c2;
            bot_fasttrig_sincos(myrect->theta-pose_dir,&s2,&c2);
            for (int p=0;p<pepper;p++) {
                erlcm_rect_t *rect = &rects->rects[rect_idx++];
                double dx=0.0;
                double dy=0.0;
                if (p<NUM_PEPPER_GRAINS/2) { 
                    dx=-0.5*myrect->size[0]+myrect->size[0]*rand()/((double)RAND_MAX);
                    dy+=0.5*myrect->size[1]*((-s2+c2)<0?-1:1);

                }
                else {
                    dx+=0.5*myrect->size[0]*((c2+s2)<0?-1:1);
                    dy=-0.5*myrect->size[1]+myrect->size[1]*rand()/((double)RAND_MAX);
                }
                rect->dxy[0]     = myrect->dxy[0] - cxyz[0] + c*dx+s*dy;
                rect->dxy[1]     = myrect->dxy[1] - cxyz[1] - s*dx+c*dy;
                rect->size[0]   = 0.27;
                rect->size[1]   = 0.27;
                rect->theta     = 0.0;// + gps_to_local.lat_lon_el_theta[3];
                rect->floor_no = myrect->floor_no;
                rect->floor_ind = myrect->floor_ind;
            }
        }
        else {
            erlcm_rect_t *rect = &rects->rects[rect_idx++];
            //double xyz[3];
            //double lle[3] = { rect->ll[0], myrect->ll[1], 0 };
            //ctrans_gps_to_local (self->ctrans, lle, xyz, NULL);
            
            rect->dxy[0]     = myrect->dxy[0] - cxyz[0];
            rect->dxy[1]     = myrect->dxy[1] - cxyz[1];
            rect->size[0]    = myrect->size[0];
            rect->size[1]    = myrect->size[1];
            rect->theta      = myrect->theta;// + gps_to_local.lat_lon_el_theta[3];
            rect->floor_no   = myrect->floor_no;
            rect->floor_ind  = myrect->floor_ind;
        }
    }
    erlcm_rect_list_t_publish(self->lcm, "SIM_RECTS", rects);   
    
    //erlcm_floor_rect_list_t_publish(self->lcm, "FLOOR_SIM_RECTS", &floor_rects);
    free(rects->rects);

    // Stop transmitting if there are no Rects
    if (rects->num_rects)
        return TRUE;
    else {
        self->transmitter = 0;
        return FALSE;
    }
}

void activate_simobs_transmitter(RendererSimObs *self)
{
    if (!self->transmitter)
        self->transmitter = g_timeout_add(1000 / TRANSMIT_OBSTACLES_HZ, on_transmit_obs, self);
}



static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererSimObs *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererSimObs *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}


static void
renderer_simobs_free(BotRenderer *super)
{
    RendererSimObs *self = (RendererSimObs*) super;
    g_array_free (self->rects, 1);
    free (super);
}


RendererSimObs *
renderer_simobs_new (BotViewer *viewer, lcm_t *_lcm, BotParam *_param )
{
    const ViewerAuxData * aux_data = get_viewer_aux_data (viewer);
    fprintf(stderr,"Called -1 ");
    
    RendererSimObs *self = (RendererSimObs*) calloc(1, sizeof(RendererSimObs));

    self->viewer = viewer;
    self->lcm = _lcm;
    //self->lcm = bot_lcm_get_global (NULL);
    BotParam *param = param;
    self->frames = bot_frames_get_global (self->lcm, param);

    self->rects = g_array_new(FALSE, FALSE, sizeof(erlcm_rect_t));
    self->edit_idx =-1;
    self->hover_idx =-1;

    self->renderer.enabled = 1;
    self->renderer.name = RENDERER_NAME;
    self->renderer.draw = renderer_simobs_draw;
    self->renderer.user = self;
    self->renderer.destroy = renderer_simobs_free;
   
    self->ehandler.name = RENDERER_NAME;
    self->ehandler.enabled = 1;
    self->ehandler.mouse_press = mouse_press;
    self->ehandler.mouse_motion = mouse_motion;
    self->ehandler.mouse_release = mouse_release;
    self->ehandler.pick_query = pick_query;
    self->ehandler.hover_query = pick_query;
    self->ehandler.key_press = key_press;
    self->ehandler.user = self;
    

    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    self->renderer.widget = GTK_WIDGET(self->pw);
    bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_PEPPER, 0, NULL);

    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);
    /*
    GtkWidget *send_obs_button = gtk_button_new_with_label ("Send Obstacles");
    gtk_box_pack_start(GTK_BOX(renderer->widget), send_obs_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(modify_goal_button), "clicked",
                     G_CALLBACK(on_send_obstacles_button), self);
    */
    erlcm_rect_list_t_subscribe(self->lcm, "SIM_RECTS_NEW", on_sim_rects_new, self);
    
    erlcm_floor_status_msg_t_subscribe(self->lcm,"CURRENT_FLOOR_STATUS",floor_status_handler, 
                                       self);

    return self;

}



void setup_renderer_simobs(BotViewer *viewer, int priority, lcm_t *_lcm, BotParam *param)
{

    RendererSimObs *self = renderer_simobs_new (viewer, _lcm, param);

    if (self) {
        bot_viewer_add_event_handler(viewer, &self->ehandler, priority);
        bot_viewer_add_renderer(viewer, &self->renderer, priority);
    }
}
