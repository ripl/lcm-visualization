/*
 * generic object (pallets + 3D object) renderer
 */

#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <GL/gl.h>
#include <gdk/gdkkeysyms.h>

#include <bot_core/bot_core.h>

//#include <bot/viewer/viewer.h>
//#include <bot/viewer/rwx.h>
//#include <bot/gl/gl_util.h>
#include "gl_utils.h"

#include <lcm/lcm.h>
//#include <common/globals.h>
//#include <common/geometry.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

//#include <common/rwx_config_model.h>
//#include <common/math_util.h>
//#include <lcmtypes/arlcm_pallet_list_t.h>
//#include <lcmtypes/arlcm_object_list_t.h>
//#include "viewer_aux_data.h"
#include <geom_utils/geometry.h>
#include "er_renderers.h"

#include <lcmtypes/hr_lcmtypes.h>
#include <hr_lcmtypes/lcm_channel_names.h>

#include <hr_common/path_util.h>

#define MODELS_DIR "test" //we should get then from path config

#ifndef MODELS_DIR
#error "MODELS_DIR is not defined!"
#endif

#define PARAM_ACTIVITY "Activity"
#define TAG_POSE "Tag Pose"
#define RENDERER_NAME "Generic Object"
#define PARAM_TRIADS "Draw Triads"
#define PARAM_MODEL "Draw Object"
#define PARAM_BBOX "Draw Bounding Boxes"
#define PARAM_PALLET_IDS "Draw Pallets IDs"
#define PARAM_OBJECT_IDS "Draw Object IDs"
#define PARAM_PEOPLE "Draw People Detections"

#if 1
#define ERR(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__); \
                      fprintf(stderr, __VA_ARGS__); fflush(stderr); } while(0)
#else
#define ERR(...) 
#endif

#if 1
#define ERR_ONCE(...) do { print_error_once(__VA_ARGS__); } while(0)
static void
print_error_once(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    char buf_stack[512];
    char *buf = buf_stack;
    int len = vsnprintf(buf, sizeof(buf_stack), format, ap);
    if (len >= sizeof(buf_stack)) {
        buf = (char*)malloc(len+1);
        vsnprintf(buf, len+1, format, ap);
    }
    if (0 == g_quark_try_string(buf)) {
        g_quark_from_string(buf);
        ERR("%s", buf);
    }
    if (buf != buf_stack)
        free(buf);
    va_end(ap);
}
#else
#define ERR_ONCE(...) 
#endif

#if 0
#define DBG(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...) 
#endif

#ifndef DRAW_UNIT_TRIADS_DEFAULT
#define DRAW_UNIT_TRIADS_DEFAULT FALSE
#endif
#ifndef DRAW_BBOX_DEFAULT
#define DRAW_BBOX_DEFAULT FALSE
#endif
#ifndef DRAW_PALLET_IDS_DEFAULT
#define DRAW_PALLET_IDS_DEFAULT FALSE
#endif
#ifndef DRAW_OBJECT_IDS_DEFAULT
#define DRAW_OBJECT_IDS_DEFAULT FALSE
#endif
#ifndef DRAW_PEOPLE_DETECTIONS_DEFAULT
#define DRAW_PEOPLE_DETECTIONS_DEFAULT TRUE
#endif


typedef struct _object_wavefront_model {
    int16_t id;
    GLuint gl_list; 
} object_wavefront_model;

typedef struct _renderer_generic_object_t {
    BotRenderer renderer;
    BotEventHandler ehandler;
    BotViewer   *viewer;
    //BotConf  *config;
    BotParam * param;
    lcm_t    *lcm;
    //RWXConf  *rwx_conf;
    //arlcm_pallet_list_t_subscription_t *pallet_lcm_hid;
    erlcm_object_list_t_subscription_t *object_lcm_hid;

    BotGtkParamWidget *pw;
    gboolean draw_unit_triads;
    gboolean draw_model;
    gboolean draw_bbox;
    gboolean draw_pallet_ids;
    gboolean draw_object_ids;
    gboolean draw_people_detections;

    GMutex *mutex; /* protect self */

    /* local copy of last pallet/object lcm message */
    //arlcm_pallet_list_t *pallet_list;
    erlcm_object_list_t *object_list;
    
    int num_of_models;
    //GLuint *gl_list;
    //we will load models as needed 
    //object_wavefront_model *model_list; 
    GHashTable *model_hash;

    gchar *last_save_filename;

    // for teleport object
    int             did_teleport; // have we done our initial teleport?
    int             teleport_request;
    uint64_t        hover_id;
    //arlcm_pallet_t *teleport_pallet;
    erlcm_object_t *teleport_object;
} renderer_generic_object_t;

//adding a mapping from type to string (name of object to object id
char *get_char_from_id(int16_t id){    
    switch(id){
    case ERLCM_OBJECT_T_TABLE:
        return "table";
        break;
    case ERLCM_OBJECT_T_CHAIR:
        return "chair";
        break;
    case ERLCM_OBJECT_T_TRASHCAN:
        return "trashcan";
        break;
    case ERLCM_OBJECT_T_BED:
        return "bed";
        break;
    case ERLCM_OBJECT_T_FRIDGE:
        return "fridge";
        break;
    case ERLCM_OBJECT_T_MICROWAVE:
        return "mircowave";
        break;
    case ERLCM_OBJECT_T_TV:
        return "tv";
        break;
    case ERLCM_OBJECT_T_ELEVATOR_DOOR:
        return "elevator_door";
        break;
    case ERLCM_OBJECT_T_LAPTOP:
        return "laptop";
        break;
    case ERLCM_OBJECT_T_WATER_FOUNTAIN:
        return "water_fountain";
        break;
    default:
        return "default";
        break;
    }
}

static GLuint
compile_display_list_extent (renderer_generic_object_t* self, char *object_name, BotWavefrontModel * model,
                              double maxv[3], double minv[3]){
    // double span_x, double span_y, double span_z)
    //{
    double span[3] = {maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2]};

    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);
    
    char key[1024];
    glPushMatrix();

    sprintf(key, "models.%s.translate", object_name);
    //double trans[3];
    //if (bot_param_get_double_array(self->param, key, trans, 3) == 3)
    //glTranslated(trans[0], trans[1], trans[2]);

    //this might be wrong - since we are scaling differntly 
    //glTranslated(-span_x/2, -span_y/2, -span_z/2);

    sprintf(key, "models.%s.scale", object_name);
    //double scale;
    //if (bot_param_get_double(self->param, key, &scale) == 0)
    //glScalef(scale, scale, scale);
    //double scale = 0.01;
    //glScalef(scale, scale, scale);
    double scale[3];
    sprintf(key, "models.%s.scale_xyz", object_name);
    if (bot_param_get_double_array(self->param, key, scale, 3) == 3) {
        glScalef (scale[0], scale[1], scale[2]);
    }

    double rot[3];
    sprintf(key, "models.%s.rotate_xyz", object_name);
    if (bot_param_get_double_array(self->param, key, rot, 3) == 3) {
      glRotatef(rot[2], 0, 0, 1);
      glRotatef(rot[1], 0, 1, 0);
      glRotatef(rot[0], 1, 0, 0);
    }

    double tf[3];
    sprintf(key, "models.%s.translate", object_name);
    if (bot_param_get_double_array(self->param, key, tf, 3) == 3) {
        glTranslated(tf[0], tf[1], tf[2]);
    }

    glEnable(GL_LIGHTING);
    bot_wavefront_model_gl_draw(model);
    glDisable(GL_LIGHTING);

    glPopMatrix();

    glEndList ();
    
    return dl;
}

static GLuint
compile_display_list_extent_2 (renderer_generic_object_t* self, char *object_name, BotWavefrontModel * model,
                              double maxv[3], double minv[3]){
    // double span_x, double span_y, double span_z)
    //{
    double span[3] = {maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2]};

    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);
    
    char key[1024];
    glPushMatrix();

    sprintf(key, "models.%s.translate", object_name);
    //double trans[3];
    //if (bot_param_get_double_array(self->param, key, trans, 3) == 3)
    //glTranslated(trans[0], trans[1], trans[2]);

    //this might be wrong - since we are scaling differntly 
    //glTranslated(-span_x/2, -span_y/2, -span_z/2);

    sprintf(key, "models.%s.scale", object_name);
    //double scale;
    //if (bot_param_get_double(self->param, key, &scale) == 0)
    //glScalef(scale, scale, scale);
    //double scale = 0.01;
    //glScalef(scale, scale, scale);
    
    

    //glScalef(1/span[0], 1/span[1], 1/span[2]);

    //glTranslated(-0.5, 0, -0.5);
    //axis are rotated
    //glTranslated(span[0]/2 - maxv[0], 0, span[2]/2 - maxv[2]);
    //glTranslated(1/2 - maxv[0]/span[0], 0/*span[1]/2 - maxv[1] */, 1/2 - maxv[2]/span[2]);
    glTranslated(1/2 - maxv[0]/span[0], 1/2 - maxv[2]/span[2], 0);
    //glTranslated(5.0, 1.0, 0);
    //glTranslated(-0.5, -0.5, 0);
    glScalef(1/span[0], 1/span[2], 1/span[1]);
    
    
    //glTranslated(0.5 - maxv[0]/span[0], 0.5 - maxv[1]/span[1], 0.5 - maxv[2]/span[2]);

    /*double rot[3];
    
    if (bot_param_get_double_array(self->param, key, rot, 3) == 3) {
      glRotatef(rot[2], 0, 0, 1);
      glRotatef(rot[1], 0, 1, 0);
      glRotatef(rot[0], 1, 0, 0);
    }

    //rotate first
    sprintf(key, "models.%s.rotate_xyz", object_name);
    fprintf(stderr, "Rot : %f,%f,%f\n", rot[0], rot[1], rot[2]);
    */
    glRotatef(90.0, 1, 0, 0);

    glEnable(GL_LIGHTING);
    bot_wavefront_model_gl_draw(model);
    glDisable(GL_LIGHTING);

    glPopMatrix();

    glEndList ();
    
    return dl;
}


static GLuint
compile_display_list_extent_1 (renderer_generic_object_t* self, char *object_name, BotWavefrontModel * model,
                              double maxv[3], double minv[3]){
    // double span_x, double span_y, double span_z)
    //{
    double span[3] = {maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2]};

    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);
    
    char key[1024];
    glPushMatrix();

    sprintf(key, "models.%s.translate", object_name);
    //double trans[3];
    //if (bot_param_get_double_array(self->param, key, trans, 3) == 3)
    //glTranslated(trans[0], trans[1], trans[2]);

    //this might be wrong - since we are scaling differntly 
    //glTranslated(-span_x/2, -span_y/2, -span_z/2);

    sprintf(key, "models.%s.scale", object_name);
    //double scale;
    //if (bot_param_get_double(self->param, key, &scale) == 0)
    //glScalef(scale, scale, scale);
    //double scale = 0.01;
    //glScalef(scale, scale, scale);
    
    glRotatef(90.0, 1, 0, 0);

    //glScalef(1/span[0], 1/span[1], 1/span[2]);

    //glTranslated(-0.5, 0, -0.5);
    //axis are rotated
    glTranslated(span[0]/2 - maxv[0], 0/*span[1]/2 - maxv[1] */, span[2]/2 - maxv[2]);
    //glTranslated(1/2 - maxv[0]/span[0], 0/*span[1]/2 - maxv[1] */, 1/2 - maxv[2]/span[2]);
    //glScalef(1/span[0], 1/span[1], 1/span[2]);
    
    
    //glTranslated(0.5 - maxv[0]/span[0], 0.5 - maxv[1]/span[1], 0.5 - maxv[2]/span[2]);

    /*double rot[3];
    
    if (bot_param_get_double_array(self->param, key, rot, 3) == 3) {
      glRotatef(rot[2], 0, 0, 1);
      glRotatef(rot[1], 0, 1, 0);
      glRotatef(rot[0], 1, 0, 0);
    }

    //rotate first
    sprintf(key, "models.%s.rotate_xyz", object_name);
    fprintf(stderr, "Rot : %f,%f,%f\n", rot[0], rot[1], rot[2]);
    */

    glEnable(GL_LIGHTING);
    bot_wavefront_model_gl_draw(model);
    glDisable(GL_LIGHTING);

    glPopMatrix();

    glEndList ();
    
    return dl;
}

static GLuint
compile_display_list (renderer_generic_object_t* self, char *object_name, BotWavefrontModel * model, double span_x, double span_y, double span_z)
{
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);
    
    char key[1024];
    glPushMatrix();

    sprintf(key, "models.%s.translate", object_name);
    //double trans[3];
    //if (bot_param_get_double_array(self->param, key, trans, 3) == 3)
    //glTranslated(trans[0], trans[1], trans[2]);

    //this might be wrong - since we are scaling differntly 
    //glTranslated(-span_x/2, -span_y/2, -span_z/2);

    sprintf(key, "models.%s.scale", object_name);
    //double scale;
    //if (bot_param_get_double(self->param, key, &scale) == 0)
    //glScalef(scale, scale, scale);
    //double scale = 0.01;
    //glScalef(scale, scale, scale);
    glScalef(1/span_x, 1/span_y, 1/span_z);
    //glScalef(0.1, 0.1, 0.1);
    
    //glTranslated(0.5, 0.5, -span_z/2 * scale);
    glTranslated(-0.5, -0.5, -0.5);

    sprintf(key, "models.%s.rotate_xyz", object_name);
    double rot[3];
    if (bot_param_get_double_array(self->param, key, rot, 3) == 3) {
      glRotatef(rot[2], 0, 0, 1);
      glRotatef(rot[1], 0, 1, 0);
      glRotatef(rot[0], 1, 0, 0);
    }

    glEnable(GL_LIGHTING);
    bot_wavefront_model_gl_draw(model);
    glDisable(GL_LIGHTING);

    glPopMatrix();

    glEndList ();
    
    return dl;
}

static GLuint
get_triad(double x, double y, double z){  
    GLuint dl = glGenLists (1);
    glNewList (dl, GL_COMPILE);

    
    /* draw a set of lines denoting current triad (helpful for debug) */
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor4f(1,0,0,1);
    glVertex3f(0., 0., 0.);
    glVertex3f(x, 0., 0.);
    glColor3f(0,1,0);
    glVertex3f(0., 0., 0.);
    glVertex3f(0., y, 0.);
    glColor3f(0,0,1);
    glVertex3f(0., 0., 0.);
    glVertex3f(0., 0., z);
    glEnd();
    glPopAttrib();
    
    glEndList();

    
    return dl;
}

static GLuint 
get_wavefront_model(char *object_name, renderer_generic_object_t *self){
    const char * models_dir = getModelsPath();
    
    char *model_name;
    char model_full_path[256];

    BotWavefrontModel *person_model;
    char param_key[1024];
    snprintf(param_key, sizeof(param_key), "models.%s.wavefront_model", object_name);
    
    if (bot_param_get_str(self->param, param_key, &model_name) == 0) {
        snprintf(model_full_path, sizeof(model_full_path), "%s/%s", models_dir, model_name);
        person_model = bot_wavefront_model_create(model_full_path);
        double minv[3];
        double maxv[3];
        bot_wavefront_model_get_extrema(person_model, minv, maxv);
        
        double span_x = maxv[0] - minv[0];
        double span_y = maxv[1] - minv[1];
        double span_z = maxv[2] - minv[2];

        //can scale this appropriately
        
        double span_max = MAX(span_x, MAX(span_y, span_z));
        
        printf("WAVEFRONT extrema: [%f, %f, %f] [%f, %f, %f]\n", 
               minv[0], minv[1], minv[2],
               maxv[0], maxv[1], maxv[2]);

        fprintf(stderr,"spans : %f,%f,%f\n", span_x, span_y, span_z);

        GLuint glcall = compile_display_list_extent(self, object_name, person_model, maxv, minv);
            
            //compile_display_list(self, object_name, person_model, span_x, span_y, span_z);

        //scale it to be 1,1,1 - and we figure out the bounding box??
        bot_wavefront_model_destroy(person_model);
        return glcall; 

        //self->person_dl = compile_display_list(self, self->person_model);
        
    }
    else {
        fprintf(stderr, "object model name not found under param %s, drawing with boxy-robot\n", param_key);
        return get_triad(1.0, 1.0, 1.0);
    }
}


static void
draw_wavefront_model (GLuint wavefront_dl, double x, double y, double heading, double scale_x, double scale_y, double scale_z)
{
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);

    glPushMatrix();
    ///x,y,z
    glTranslated(x, y, 0);
    //glTranslated(5, 2, 0);
    //orientation
    glRotatef(heading, 0, 0, 1);
    
    //glRotatef(30, 0, 0, 1);
    
    glScalef(scale_x, scale_y, scale_z);
    //glScalef(.5,.5,.5);
    
    glCallList (wavefront_dl);
    //undo 
    glPopMatrix();    
}

/**
 * draw a set of lines denoting current triad (helpful for debug)
 */
static void
draw_unit_triad()
{  
    
    static GLuint dl = 0;
    if (!dl) {
        dl = glGenLists(1);
        if (dl) {
            glNewList(dl, GL_COMPILE);
            DBG("Compiling unit triad display list\n");
        }

        /* draw a set of lines denoting current triad (helpful for debug) */
        glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        glColor4f(1,0,0,1);
        glVertex3f(0., 0., 0.);
        glVertex3f(1., 0., 0.);
        glColor3f(0,1,0);
        glVertex3f(0., 0., 0.);
        glVertex3f(0., 1., 0.);
        glColor3f(0,0,1);
        glVertex3f(0., 0., 0.);
        glVertex3f(0., 0., 1.);
        glEnd();
        glPopAttrib();

        if (dl) {
            glEndList();
        }
    }
    if (dl) {
        glCallList(dl);
    }
}

/**
 * 
 */
static void
draw_unit_cube(double red, double green, double blue)
{
    /* Attributes */
    glPushAttrib(GL_CURRENT_BIT | GL_LIGHTING_BIT);
    glEnable(GL_LIGHTING);

    float ambient = 0.4;
    float diffuse = 0.6;
    float specular =  0.;
    float opacity = 1;
    int shininess = 20;
    float temp_color[4];
    /* ambient */
    temp_color[0] = red * ambient;
    temp_color[1] = green * ambient;
    temp_color[2] = blue * ambient;
    temp_color[3] = opacity;
    glMaterialfv(GL_FRONT, GL_AMBIENT, temp_color);
    /* diffuse */
    temp_color[0] = red * diffuse;
    temp_color[1] = green * diffuse;
    temp_color[2] = blue * diffuse;
    temp_color[3] = opacity;
    glMaterialfv(GL_FRONT, GL_DIFFUSE, temp_color );
    /* specular */
    temp_color[0] = red * specular;
    temp_color[1] = green * specular;
    temp_color[2] = blue * specular;
    temp_color[3] = opacity;
    glMaterialfv(GL_FRONT, GL_SPECULAR, temp_color);
    /* emission */
    temp_color[0] = temp_color[1] = temp_color[2] = 0;
    temp_color[3] = opacity;
    glMaterialfv(GL_FRONT, GL_EMISSION, temp_color );
    /* shininess */
    glMateriali(GL_FRONT, GL_SHININESS, shininess);

    static GLuint dl = 0;
    if (!dl) {
        dl = glGenLists(1);
        if (dl) {
            glNewList(dl, GL_COMPILE);
            DBG("Compiling unit cube display list\n");
        }

        glBegin(GL_QUADS);

        glNormal3f(0., 0., -1.);
        glVertex3f(0., 1., 0.);
        glVertex3f(0., 0., 0.);
        glVertex3f(1., 0., 0.);
        glVertex3f(1., 1., 0.);

        glNormal3f(0., -1., 0.);
        glVertex3f(0., 0., 0.);
        glVertex3f(0., 0., 1.);
        glVertex3f(1., 0., 1.);
        glVertex3f(1., 0., 0.);

        glNormal3f(1., 0., 0.);
        glVertex3f(1., 1., 0.);
        glVertex3f(1., 0., 0.);
        glVertex3f(1., 0., 1.);
        glVertex3f(1., 1., 1.);

        glNormal3f(0., 0., 1.);
        glVertex3f(1., 1., 1.);
        glVertex3f(1., 0., 1.);
        glVertex3f(0., 0., 1.);
        glVertex3f(0., 1., 1.);

        glNormal3f(-1., 0., 0.);
        glVertex3f(0., 1., 1.);
        glVertex3f(0., 0., 1.);
        glVertex3f(0., 0., 0.);
        glVertex3f(0., 1., 0.);

        glNormal3f(0., 1., 0.);
        glVertex3f(1., 1., 1.);
        glVertex3f(0., 1., 1.);
        glVertex3f(0., 1., 0.);
        glVertex3f(1., 1., 0.);

        glEnd();

        if (dl) {
            glEndList();
        }
    }
    if (dl) {
        glCallList(dl);
    }

    glPopAttrib();
}

/**
 * 
 */
static void
draw_wireframe_unit_cube(double red, double green, double blue)
{
    /* draw a set of lines denoting current triad (helpful for debug) */
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glColor4d(red,green,blue,1);

    static GLuint dl = 0;
    if (!dl) {
        dl = glGenLists(1);
        if (dl) {
            glNewList(dl, GL_COMPILE);
            DBG("Compiling unit wireframe object display list\n");
        }

        glBegin(GL_LINES);
        /* bottom 4 */
        glVertex3f(0., 0., 0.);
        glVertex3f(1., 0., 0.);
        glVertex3f(1., 0., 0.);
        glVertex3f(1., 1., 0.);
        glVertex3f(1., 1., 0.);
        glVertex3f(0., 1., 0.);
        glVertex3f(0., 1., 0.);
        glVertex3f(0., 0., 0.);
        /* top 4 */
        glVertex3f(0., 0., 1.);
        glVertex3f(1., 0., 1.);
        glVertex3f(1., 0., 1.);
        glVertex3f(1., 1., 1.);
        glVertex3f(1., 1., 1.);
        glVertex3f(0., 1., 1.);
        glVertex3f(0., 1., 1.);
        glVertex3f(0., 0., 1.);
        /* side 4 */
        glVertex3f(0., 0., 0.);
        glVertex3f(0., 0., 1.);
        glVertex3f(1., 0., 0.);
        glVertex3f(1., 0., 1.);
        glVertex3f(1., 1., 0.);
        glVertex3f(1., 1., 1.);
        glVertex3f(0., 1., 0.);
        glVertex3f(0., 1., 1.);
        glEnd();

        if (dl) {
            glEndList();
        }
    }
    if (dl) {
        glCallList(dl);
    }

    glPopAttrib();
}

/*
 * Transform the OpenGL modelview matrix such that a unit cube maps to
 * the bounding box specified by min_v and max_v.
 */
static void
transform_bounding_box(const double min_v[3], const double max_v[3])
{
    glTranslated(min_v[0], min_v[1], min_v[2]);
    glScaled(max_v[0] - min_v[0], 
             max_v[1] - min_v[1], 
             max_v[2] - min_v[2]);
}

/*static void
draw_pallet(renderer_generic_object_t *self, const erlcm_pallet_t *pallet)
{
    // If we are in simulation mode, don't draw pallets with id = -1,
    // which correspond to pallets added by the pallet filter, since they
    // already exist.
    if (self->viewer != NULL) {
        const ViewerAuxData * aux_data = get_viewer_aux_data (self->viewer);
        if (aux_data->simulation_flag && (pallet->id == -1))
            return;
    }

    // get the model 
    GString *config_prefix = g_string_new("");
    switch (pallet->label) {
    case ARLCM_PALLET_T_LABEL_TIRES:
        g_string_printf(config_prefix, "pallet_rwx.6");
        break;
    case ARLCM_PALLET_T_LABEL_TIRE_ASSEMBLY:
        g_string_printf(config_prefix, "pallet_rwx.6");
        break;
    case ARLCM_PALLET_T_LABEL_EMPTY_WOODEN:
        g_string_printf(config_prefix, "pallet_rwx.4");
        break;
    case ARLCM_PALLET_T_LABEL_GENERATOR_ACCESSORIES:
        g_string_printf(config_prefix, "pallet_rwx.5");
        break;
    default:
        g_string_printf(config_prefix, "pallet_rwx.4");
        break;
    }

    RWXConfModel *model = 
        rwx_conf_get_model(self->rwx_conf, config_prefix->str);

    // compute the rotation matrix, then transpose to get OpenGL
     // column-major matrix 
    double m[16], m_opengl[16];
    bot_quat_pos_to_matrix(pallet->orientation, pallet->pos, m);
    bot_matrix_transpose_4x4d(m, m_opengl);

    // is the bounding box valid? 
    gboolean bbox_valid = (pallet->bbox_max[0] != pallet->bbox_min[0] &&
                           pallet->bbox_max[1] != pallet->bbox_min[1] &&
                           pallet->bbox_max[2] != pallet->bbox_min[2]);

    if (model) {
        glPushMatrix();
        glMultMatrixd(m_opengl);

        if (bbox_valid && 
            (self->draw_bbox||pallet->id==self->hover_id)) {
            glPushMatrix();
            transform_bounding_box(pallet->bbox_min, pallet->bbox_max);
            if (pallet->id==self->hover_id)
                draw_wireframe_unit_cube(1.0,0.5,0.83); // ???? 
            else
                draw_wireframe_unit_cube(0.5,1,0.83); // aquamarine 
            glPopMatrix();
        }
        if (self->draw_unit_triads)
            draw_unit_triad();
// draw the pallet 3D model 
        rwx_conf_model_gl_draw(model);

        glPopMatrix();
        
        // draw pallet slots
        glColor4f (1, 1, 1, 1);
        glBegin (GL_LINES);
        for(int spind=0; spind<pallet->num_slot_pairs; spind++) {
            arlcm_pallet_slot_pair_t *sp = &pallet->slot_pairs[spind];
            arlcm_pallet_slot_t *slots[2] = {
                &sp->left_slot,
                &sp->right_slot
            };
            for(int slotind=0; slotind<2; slotind++) {
                arlcm_pallet_slot_t *slot = slots[slotind];

                // transform slot corners to local frame
                // Z axis of "slot" frame == slot->dir in local frame
                double up_local[3] = { 0, 0, 1 };
                double sleft[3], sup[3];
                bot_vector_cross_3d(up_local, slot->dir, sleft);
                bot_vector_cross_3d(slot->dir, sleft, sup);
                bot_vector_normalize_3d(sleft);
                bot_vector_normalize_3d(sup);
                glPushMatrix();
                glTranslated(slot->pos[0], slot->pos[1], slot->pos[2]);

                bot_vector_scale_3d(sleft, slot->size[0]/2);
                bot_vector_scale_3d(sup, slot->size[1]/2);

    glVertex3d(-sleft[0] -sup[0], -sleft[1] -sup[1], -sleft[2] -sup[2]);
    glVertex3d( sleft[0] +sup[0],  sleft[1] +sup[1],  sleft[2] +sup[2]);
    glVertex3d( sleft[0] -sup[0],  sleft[1] -sup[1],  sleft[2] -sup[2]);
    glVertex3d(-sleft[0] +sup[0], -sleft[1] +sup[1], -sleft[2] +sup[2]);
                glPopMatrix();
            }
        }
        glEnd();
    }
    else if (bbox_valid) {
        // no model but the bounding box is valid 
        glPushMatrix();
        glMultMatrixd(m_opengl);
        if (self->draw_unit_triads)
            draw_unit_triad();
        transform_bounding_box(pallet->bbox_min, pallet->bbox_max);
        draw_unit_cube(0.5,1,0.83); // aquamarine 
        glPopMatrix();
    }
    else {
    // let someone know that we are not rendering a pallet 
        ERR_ONCE("Error: unable to draw pallet of type '%s', pallet has no rwx "
                 "model or valid bounding box\n", config_prefix->str);
    }

    if (self->draw_pallet_ids) {
        char id[256];
        sprintf(id, "%"PRId64, pallet->id);
        glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
        glDisable(GL_LIGHTING);
        glColor4f(1,1,1,1);
        bot_gl_draw_text(pallet->pos, GLUT_BITMAP_HELVETICA_12, id,
        0);
        glPopAttrib();
    }

    g_string_free(config_prefix, TRUE);
}*/

static void
draw_object(renderer_generic_object_t *self, const erlcm_object_t *object)
{
    /* get the model */
    GString *config_prefix = g_string_new("");
    g_string_printf(config_prefix, "object_wavefront.%d", object->object_type);
    char *name = get_char_from_id(object->object_type);

    gpointer found_model = NULL;

    GLuint *gl_model = NULL;//calloc(1,sizeof(GLuint));//NULL;

    GLuint *gl_triad = get_triad(1.0, 1.0, 1.0);
    //*gl_model = get_wavefront_model(name, self);
   
    
    //fprintf(stderr,"Looking for model\n");
    if(g_hash_table_lookup_extended(self->model_hash, name, NULL, 
                                    &found_model)){
        //fprintf(stderr,"Found model\n");
        gl_model = (GLuint *)found_model;
    }
    else{
        //create gl calls
        //fprintf(stderr,"creating gl calls\n");
        
        GLuint *new_gl_model = calloc(1,sizeof(GLuint));
        *new_gl_model = get_wavefront_model(name, self);        
        char *conf_path_copy = g_strdup(name);
        g_hash_table_insert(self->model_hash, conf_path_copy, new_gl_model);        
        gl_model = new_gl_model;
    }

    /* compute the rotation matrix, then transpose to get OpenGL
     * column-major matrix */
    double m[16], m_opengl[16];
    bot_quat_pos_to_matrix(object->orientation, object->pos, m);
    bot_matrix_transpose_4x4d(m, m_opengl);

    /* is the bounding box valid? */
    gboolean bbox_valid = (object->bbox_max[0] != object->bbox_min[0] &&
                           object->bbox_max[1] != object->bbox_min[1] &&
                           object->bbox_max[2] != object->bbox_min[2]);

    //our call doesnt have valid bbox 

    if (0 /*model*/) {
        glPushMatrix();
        glMultMatrixd(m_opengl);

        if (bbox_valid &&
            (self->draw_bbox||object->id==self->hover_id)) {
            glPushMatrix();
            transform_bounding_box(object->bbox_min, object->bbox_max);
            if (object->id==self->hover_id)
                draw_wireframe_unit_cube(1.0,0.5,0.83); /* ???? */
            else
                draw_wireframe_unit_cube(1, 0.5, 0); /* orange */
            glPopMatrix();
        }
        if (self->draw_unit_triads)
            draw_unit_triad();
        /* draw the object 3D model */
        //rwx_conf_model_gl_draw(model);
        glPopMatrix();
    }
    else if (bbox_valid) {
        /* no model but the bounding box is valid */
        glPushMatrix();
        glMultMatrixd(m_opengl);

        if (self->draw_unit_triads){// && gl_model)
            //draw_wavefront_model(*gl_triad, 0,0,0,1,1,1);
            draw_unit_triad();
        }

        if(self->draw_model){
            draw_wavefront_model(*gl_model/*self->gl_list[1]*/, 0,0,0,1,1,1);
        }

        transform_bounding_box(object->bbox_min, object->bbox_max);
        draw_unit_cube(1, 0.5, 0); /* orange */
        glPopMatrix();
    }
    else {
        /* let someone know that we are not rendering a object */
        ERR_ONCE("Error: unable to draw object of type '%s', object has no rwx "
                 "model or valid bounding box\n", config_prefix->str);
    }

    if (self->draw_object_ids) {
        char id[256];
        sprintf(id, "%"PRId64, object->id);
        glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
        glDisable(GL_LIGHTING);
        glColor4f(1,1,1,1);
        bot_gl_draw_text(object->pos, GLUT_BITMAP_HELVETICA_12, id,
                         0/*BOT_GL_DRAW_TEXT_DROP_SHADOW*/);
        glPopAttrib();
    }

    g_string_free(config_prefix, TRUE);
}

static void
on_object_list(const lcm_recv_buf_t *rbuf, const char *channel,
               const erlcm_object_list_t *msg, void *user)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*)user;

    /* copy lcm message data to local buffer and let draw update the display */
    g_mutex_lock(self->mutex);
    if (self->object_list)
        erlcm_object_list_t_destroy(self->object_list);
    self->object_list = erlcm_object_list_t_copy(msg);
    BotViewer *viewer = self->viewer; /* copy viewer to stack in case self is 
                                    * free'd between the unlock and call to 
                                    * viewer_request_redraw */
    g_mutex_unlock(self->mutex); /* not sure whether viewer_request_redraw 
                                  * calls this renderer's draw function before 
                                  * returning. therefore, I am releaseing the 
                                  * mutex here to avoid possibility of a 
                                  * deadlock */

    if (viewer)
        bot_viewer_request_redraw(viewer);
}

char *
objects_snapshot (renderer_generic_object_t *self)
{
    GString *result = 
        g_string_new ("<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n");

    //convert this to bot trans
    double lat_lon_el[3];
    double theta;
    double rpy[3];
    
    g_string_append (result, "<objects>\n");

    int count = 0;
    
    // loop through the object list
    count = 0;
    if (self->object_list && self->object_list->num_objects) {
        for (int i=0; i<self->object_list->num_objects; i++) {
            erlcm_object_t *o = self->object_list->objects + i;
            if (o->id == -1)
                continue;

            g_string_append (result,"    <item type=\"object\">\n");
            g_string_append_printf (result, 
                                    "        <id>%"PRId64"</id>\n", o->id);
            g_string_append_printf (result, 
                                    "        <type id=\"object_enum_t\">%d</type>\n", o->object_type);
            g_string_append_printf (result, 
                                    "        <position id=\"local-frame\">%lf %lf %lf</position>\n", 
                                    o->pos[0], o->pos[1], o->pos[2]);

            bot_quat_to_roll_pitch_yaw (o->orientation, rpy);
            theta = 0;// rpy[2] - gps_to_local.lat_lon_el_theta[3];
 
            //atrans_local_to_gps (atrans, o->pos, lat_lon_el, NULL);
            lat_lon_el[2] = 0.0;

            g_string_append_printf (result, 
                                    "        <position id=\"lat_lon_theta\">%lf %lf %lf</position>\n", 
                                    lat_lon_el[0], lat_lon_el[1], theta);

            g_string_append_printf (result, 
                                    "        <orientation id=\"local-frame\">%lf %lf %lf %lf</orientation>\n", 
                                    o->orientation[0], o->orientation[1], o->orientation[2], o->orientation[3]);
        
            g_string_append (result, "    </item>\n");
            count += 1;
        }
    }
    fprintf(stdout,"Writing %d generic objects\n", count);
    //globals_release_atrans (atrans);        
    g_string_append (result, "</objects>\n");
    return g_string_free (result, FALSE);
}

static void
on_load_button (GtkWidget *button, renderer_generic_object_t *self)
{
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new("Save Objects", NULL,
                                         GTK_FILE_CHOOSER_ACTION_SAVE,
                                         GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                                         GTK_STOCK_SAVE, GTK_RESPONSE_ACCEPT,
                                         NULL);
    
    if (self->last_save_filename)
        gtk_file_chooser_set_filename (GTK_FILE_CHOOSER(dialog),
                                       self->last_save_filename);

    if (gtk_dialog_run (GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char *filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
        if (filename != NULL) {
            if (self->last_save_filename)
                g_free (self->last_save_filename);
            self->last_save_filename = g_strdup (filename);

            //send xml write command to the object simulator 

            erlcm_xml_cmd_t msg; 
            msg.utime = bot_timestamp_now();
            msg.cmd_type = ERLCM_XML_CMD_T_LOAD_FILE; 
            msg.path = strdup(filename);
            erlcm_xml_cmd_t_publish(self->lcm, "XML_COMMAND", &msg);

            free (filename);
            free(msg.path);
        }
    }
   
    gtk_widget_destroy (dialog);
}


static void
on_save_button (GtkWidget *button, renderer_generic_object_t *self)
{
    GtkWidget *dialog;
    dialog = gtk_file_chooser_dialog_new("Save Objects", NULL,
                                         GTK_FILE_CHOOSER_ACTION_SAVE,
                                         GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                                         GTK_STOCK_SAVE, GTK_RESPONSE_ACCEPT,
                                         NULL);
    
    if (self->last_save_filename)
        gtk_file_chooser_set_filename (GTK_FILE_CHOOSER(dialog),
                                       self->last_save_filename);

    if (gtk_dialog_run (GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char *filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
        if (filename != NULL) {
            if (self->last_save_filename)
                g_free (self->last_save_filename);
            self->last_save_filename = g_strdup (filename);

            //send xml write command to the object simulator 

            erlcm_xml_cmd_t msg; 
            msg.utime = bot_timestamp_now();
            msg.cmd_type = ERLCM_XML_CMD_T_WRITE_FILE; 
            msg.path = strdup(filename);

            erlcm_xml_cmd_t_publish(self->lcm, "XML_COMMAND", &msg);
           
            free (filename);
            free(msg.path);
        }
    }
   
    gtk_widget_destroy (dialog);
}

static void
renderer_generic_object_draw(BotViewer *viewer, BotRenderer *renderer)
{
    // Needed for N810 viewer so the textures get rendered correctly.
    if (!viewer) glDisable(GL_COLOR_MATERIAL);

    renderer_generic_object_t *self = renderer->user;

    g_mutex_lock(self->mutex);
    if (self->object_list && self->object_list->num_objects){

        /* set desired DL attributes */
        glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | 
                     GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | 
                     GL_LIGHTING_BIT | GL_TRANSFORM_BIT);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_RESCALE_NORMAL);
        glShadeModel(GL_SMOOTH);
        glEnable(GL_LIGHTING);

        /* iterate through object lists, drawing each */
     
        if (self->object_list && self->object_list->num_objects) {
            for (int i = 0; i < self->object_list->num_objects; i++) {         
                erlcm_object_t *object = self->object_list->objects + i;
                
                draw_object(self, self->object_list->objects + i);
                
                //add the drawing  call here 
		/*if ((object->object_type ==  ERLCM_OBJECT_ENUM_T_OPERATOR && self->draw_people_detections) ||
		    object->object_type != ARLCM_OBJECT_ENUM_T_OPERATOR)
		    draw_object(self, self->object_list->objects + i);*/
            }
        }

        /* return to original attributes */
        glPopAttrib();
    }
    g_mutex_unlock(self->mutex);

    if (!viewer) glEnable(GL_COLOR_MATERIAL);
}

static void
renderer_generic_object_destroy(BotRenderer *renderer)
{
    if (!renderer)
        return;

    renderer_generic_object_t *self = renderer->user;
    if (!self)
        return;

    if (self->mutex)
        g_mutex_lock(self->mutex);
    
    /* stop listening to lcm */
    if (self->lcm) {
        if (self->object_lcm_hid)
            erlcm_object_list_t_unsubscribe(self->lcm, 
                                            self->object_lcm_hid);
    }
    
    /* destory local copy of lcm data objects */
    if (self->object_list)
        erlcm_object_list_t_destroy(self->object_list);
    
    if (self->last_save_filename)
        g_free (self->last_save_filename);

    /* release mutex */
    if (self->mutex) {
        g_mutex_unlock(self->mutex);
        g_mutex_free(self->mutex);
    }
    
    free(self);
}

static void 
on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*)user;

    g_mutex_lock(self->mutex);
    
    if(!strcmp(name, TAG_POSE)) {
        erlcm_affordance_tag_t msg;
        msg.utime = bot_timestamp_now();
        msg.source = ERLCM_AFFORDANCE_TAG_T_SOURCE_VIEWER;
        msg.activity = bot_gtk_param_widget_get_enum(self->pw, PARAM_ACTIVITY);
        erlcm_affordance_tag_t_publish(self->lcm, "AFFORDANCE_TAG", &msg);
    }

    self->draw_unit_triads = 
        bot_gtk_param_widget_get_bool(self->pw, PARAM_TRIADS);
    self->draw_model = 
        bot_gtk_param_widget_get_bool(self->pw, PARAM_MODEL);
    self->draw_bbox = 
        bot_gtk_param_widget_get_bool(self->pw, PARAM_BBOX);
    self->draw_pallet_ids = 
        bot_gtk_param_widget_get_bool(self->pw, PARAM_PALLET_IDS);
    self->draw_object_ids = 
        bot_gtk_param_widget_get_bool(self->pw, PARAM_OBJECT_IDS);
    self->draw_people_detections = 
        bot_gtk_param_widget_get_bool(self->pw, PARAM_PEOPLE);
    g_mutex_unlock(self->mutex);

    bot_viewer_request_redraw(self->viewer);
}

static void 
on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*)user;
    bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void 
on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*)user;
    bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}


static erlcm_object_t * 
find_closest_object(renderer_generic_object_t *self, const double pos[3])

{
    double closest_dist=HUGE;
    erlcm_object_t *closest_object=NULL;
    /* iterate through object list to find closest object to pos */
    if (self->object_list && self->object_list->num_objects) {
        for (int i = 0; i < self->object_list->num_objects; i++) {
            erlcm_object_t *p = self->object_list->objects + i;  
            double dist = sqrt(bot_sq(p->pos[0] - pos[0]) + bot_sq(p->pos[1] - pos[1]));        
            if (closest_dist>dist) {
                closest_dist=dist;
                closest_object=p;
            }
        }
        
    }
    return closest_object;
}


static double pick_query(BotViewer *viewer, BotEventHandler *ehandler, 
                         const double ray_start[3], const double ray_dir[3])
{
    renderer_generic_object_t *self = (renderer_generic_object_t*) ehandler->user;
    if (!self->teleport_request)
       return -1;

    double pos[3]={0,0,0};
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
            POINT3D(ray_dir), 0, POINT2D(pos));

    g_mutex_lock(self->mutex);

    double closest_dist =HUGE;
    erlcm_object_t *closest_object = find_closest_object(self,pos);
    if (closest_object) {
        double dist = sqrt(bot_sq(closest_object->pos[0] - pos[0]) + bot_sq(closest_object->pos[1] - pos[1]));
        if (dist<closest_dist) {
            self->hover_id = closest_object->id;
            closest_dist=dist;
        }
    }
    g_mutex_unlock(self->mutex);
    if (closest_dist !=HUGE) 
        return closest_dist;

    self->ehandler.hovering = 0;
    return -1;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler,
                        const double ray_start[3], const double ray_dir[3], 
                        const GdkEventButton *event)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*) ehandler->user;

    /*double pos[3]={0,0,0};
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
                                  POINT3D(ray_dir), 0, POINT2D(pos));
    
                                  fprintf(stderr, "Pos : %f, %f,%f\n", pos[0], pos[1], pos[2]);*/

    // only handle mouse button 1.
    if (self->teleport_request && (event->button == 1)) {   
        // find teleport object
        double pos[3]={0,0,0};
        geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
                                      POINT3D(ray_dir), 0, POINT2D(pos));

        g_mutex_lock(self->mutex);
        double closest_pallet_dist =HUGE;
        double closest_object_dist =HUGE;
 
        erlcm_object_t *closest_object = find_closest_object(self,pos);
        if (closest_object) {
            closest_object_dist = sqrt(bot_sq(closest_object->pos[0] - pos[0]) + bot_sq(closest_object->pos[1] - pos[1]));
        }
        if (closest_object&&closest_object_dist<closest_pallet_dist) {
            self->hover_id = closest_object->id;
            if (self->teleport_object)
                erlcm_object_t_destroy(self->teleport_object);
            self->teleport_object = erlcm_object_t_copy(closest_object);
        }
        g_mutex_unlock(self->mutex);
        //self->teleport_object = 1;
        //self->teleport_pallet = 1;
    }
    return 0;
}

static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventButton *event)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*) ehandler->user;

    self->hover_id=0;
    self->teleport_request = 0;
    self->ehandler.picking = 0;
    if (self->teleport_object) {
        erlcm_object_t_destroy(self->teleport_object);
        self->teleport_object=NULL;
    }

    return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventMotion *event)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*) ehandler->user;
    
    /*const ViewerAuxData * aux_data = get_viewer_aux_data (viewer);
    if (!aux_data->simulation_flag||!self->teleport_request)
    return 0;*/
    if (!self->teleport_request)
        return 0;

    if (!self->teleport_object) {
        pick_query(viewer, ehandler, ray_start, ray_dir);
        return 0;
    }

    double pos[3];
    if (self->teleport_object) 
        memcpy(pos,self->teleport_object->pos, 3 * sizeof(double));

    //fprintf(stderr,"Pos : %f,%f,%f\n", pos[0], pos[1], pos[2]);

    double v = (pos[2] - ray_start[2]) / ray_dir[2];
    double xy[2];
    xy[0] = ray_start[0] + v * ray_dir[0];
    xy[1] = ray_start[1] + v * ray_dir[1];
    
    int shift = event->state & GDK_SHIFT_MASK;

    if (shift) {
        // rotate!
        double theta1 = atan2(xy[1] - pos[1], xy[0] - pos[0]);

        // local
        double dq[4] = { cos (theta1/2), 0, 0, sin(theta1/2) };
        if (self->teleport_object) 
            memcpy(self->teleport_object->orientation, dq, 4 * sizeof(double));

    } else {
        // translate
        if (self->teleport_object) {
            self->teleport_object->pos[0] = xy[0];
            self->teleport_object->pos[1] = xy[1];
        }
    }
    if (self->teleport_object) {
        erlcm_object_list_t ol;
        ol.num_objects=1;
        ol.utime = bot_timestamp_now();
        self->teleport_object->utime=ol.utime;
        ol.objects = self->teleport_object;
        erlcm_object_list_t_publish(self->lcm, "OBJECTS_UPDATE_RENDERER", &ol);
    }

    return 1;
}


static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
        const GdkEventKey *event)
{
    renderer_generic_object_t *self = (renderer_generic_object_t*) ehandler->user;

    if (event->keyval == 's' || event->keyval == 'S') {
        bot_viewer_request_pick(viewer, ehandler);
        self->teleport_request = 1;
        return 1;
    }
    if (event->keyval == GDK_Escape) {
        ehandler->picking = 0;
        self->teleport_request = 0;
        self->hover_id=0;
        if (self->teleport_object) {
            erlcm_object_t_destroy(self->teleport_object);
            self->teleport_object=NULL;
        }
    }

    return 0;
}

BotRenderer*
renderer_generic_object_new(BotViewer *viewer, BotParam * param)
{
    renderer_generic_object_t *self = 
        (renderer_generic_object_t*)calloc(1,sizeof(renderer_generic_object_t));
    if (!self) {
        ERR("Error: renderer_generic_object_new() failed to allocate self\n");
        goto fail;
    }

    self->renderer.draw = renderer_generic_object_draw;
    self->renderer.destroy = renderer_generic_object_destroy;
    self->renderer.user = self;
    self->pw = NULL;
    self->renderer.widget = NULL;
    self->renderer.name = RENDERER_NAME;
    self->renderer.enabled = 1;


    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->key_press = key_press;
    ehandler->hover_query = pick_query;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->user = self;


    /* global objects */
    self->viewer = viewer;
    //add bot param 
    self->param = param;

    self->lcm = bot_lcm_get_global (NULL);//globals_get_lcm();
    if (!self->lcm) {
        ERR("Error: renderer_generic_object_new() failed to get global lcm "
            "object\n");
        goto fail;
    }

    /* mutex:
     * lock the mutex within the following functions:
     *   on_pallet_list
     *   on_object_list
     *   on_param_widget_changed
     *   renderer_generic_object_draw
     *   renderer_generic_object_destroy
     */
    self->mutex = g_mutex_new();
    if (!self->mutex) {
        ERR("Error: renderer_generic_object_new() failed to create the "
            "generic object renderer mutex\n");
        goto fail;
    }

    /* listen to object list */

    self->object_lcm_hid = erlcm_object_list_t_subscribe(self->lcm, 
        "OBJECT_LIST", on_object_list, self);
    if (!self->object_lcm_hid) {
        ERR("Error: renderer_generic_object_new() failed to subscribe to the "
            "'OBJECT_LIST' LCM channel\n");
        goto fail;
    }

    /* renderer options defaults */
    self->draw_unit_triads = DRAW_UNIT_TRIADS_DEFAULT;
    self->draw_bbox = DRAW_BBOX_DEFAULT;
    self->draw_pallet_ids = DRAW_PALLET_IDS_DEFAULT;
    self->draw_object_ids = DRAW_OBJECT_IDS_DEFAULT;
    self->draw_people_detections = DRAW_PEOPLE_DETECTIONS_DEFAULT;
    
    /*self->num_of_models = 5;
    self->gl_list = (GLuint *)calloc(self->num_of_models, sizeof(GLuint));

    

    for(int i=0; i < self->num_of_models; i++){        
        self->gl_list[i] = get_wavefront_model("chair", self);
        }*/

    self->model_hash = g_hash_table_new_full(g_str_hash, g_str_equal, 
                                             free, free);//bot_wavefront_model_destroy);
    //_rwx_conf_model_destroy);

    
    if (viewer) {
        self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
        self->renderer.widget = GTK_WIDGET(self->pw);

        /* setup parameter widget */
        bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_TRIADS, 
                                          self->draw_unit_triads, NULL);
        bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_MODEL,             
                                          self->draw_unit_triads, NULL);

        bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_BBOX, 
                                          self->draw_bbox, NULL);
        bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_PALLET_IDS, 
                                          self->draw_pallet_ids, NULL);
        bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_OBJECT_IDS, 
                                          self->draw_object_ids, NULL);
        bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_PEOPLE, 
                                          self->draw_people_detections, NULL);

        //
        bot_gtk_param_widget_add_enum(self->pw, PARAM_ACTIVITY, BOT_GTK_PARAM_WIDGET_MENU, 
                                      0, 
                                      "Watching TV", ERLCM_AFFORDANCE_TAG_T_ACTIVITY_WATCHING_TV,
                                      "Eating Dinner", ERLCM_AFFORDANCE_TAG_T_ACTIVITY_EATING_DINNER,
                                      "Using computer", ERLCM_AFFORDANCE_TAG_T_ACTIVITY_USING_COMPUTER,
                                      "Painting", ERLCM_AFFORDANCE_TAG_T_ACTIVITY_PAINTING,
                                      "Reading", ERLCM_AFFORDANCE_TAG_T_ACTIVITY_READING,
                                      NULL);
        

        bot_gtk_param_widget_add_buttons(self->pw, TAG_POSE, NULL);

        GtkWidget *save_button = gtk_button_new_with_label("Save to xml");
        gtk_box_pack_start (GTK_BOX(self->renderer.widget), save_button,
                            FALSE, FALSE, 0);
        g_signal_connect (G_OBJECT(save_button), "clicked",
                          G_CALLBACK(on_save_button), self);

        GtkWidget *load_button = gtk_button_new_with_label("Load from xml");
        gtk_box_pack_start (GTK_BOX(self->renderer.widget), load_button,
                            FALSE, FALSE, 0);
        g_signal_connect (G_OBJECT(load_button), "clicked",
                          G_CALLBACK(on_load_button), self);

        
        gtk_widget_show_all (self->renderer.widget);

        /* setup viewer signal callbacks */
        g_signal_connect(G_OBJECT(self->pw), "changed", 
                         G_CALLBACK(on_param_widget_changed), self);
        g_signal_connect(G_OBJECT(viewer), "load-preferences", 
                         G_CALLBACK(on_load_preferences), self);
        g_signal_connect(G_OBJECT(viewer), "save-preferences",
                         G_CALLBACK(on_save_preferences), self);
    }

    return &self->renderer;
 fail:
    renderer_generic_object_destroy(&self->renderer);
    return NULL;
}




void
setup_renderer_generic_object(BotViewer *viewer, int render_priority, BotParam * param)
{
    BotRenderer *renderer = renderer_generic_object_new(viewer, param);
    renderer_generic_object_t *self = renderer->user;

    if (viewer && renderer &&self) {
        /* add renderer iff viewer exists */
        bot_viewer_add_renderer(viewer, renderer, render_priority);
        bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
    }
}
