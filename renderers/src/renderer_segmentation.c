#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gsl/gsl_fit.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_blas.h>

#include <bot/bot_core.h>
#include <bot/gl/gl_util.h>
#include <bot/gtk/gtk_util.h>
#include <bot/viewer/viewer.h>
#include <lcm/lcm.h>
#include <common3d/agile-arconf.h>
#include <common3d/agile-color_util.h>
#include <common3d/ripl_trans.h>
#include <common3d/agile-globals.h>
#include <common3d/agile-geometry.h>
#include <common3d/laser_utils.h>
#include <common3d/lcm_channel_names.h>
#include <lcmtypes/wheelchair3d_lcmtypes.h>

#define RENDERER_NAME "Segmentation"
#define PARAM_SCAN_MEMORY "Scan Memory"
#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_Z_BUFFER "Z Buffer"
#define PARAM_BIG_POINTS "Big Points"
#define PARAM_SPATIAL_DECIMATE "Spatial Decimation"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"

#define MAX_SCAN_MEMORY 10000
#define OLD_HISTORY_THRESHOLD 3000000 /* microseconds */
#define MAX_SENSOR_RANGE_DEFAULT 30.0 /* meters */
#define MIN_SENSOR_RANGE_DEFAULT 0.15 /* meters */
#define COLOR_MODE_Z_MAX_Z 7.5
#define COLOR_MODE_Z_MIN_Z -.5
#define COLOR_MODE_Z_DZ 0.01
#define SPACIAL_DECIMATION_LIMIT 0.05 /* meters */

#ifndef DATA_FROM_LR3
#define DATA_FROM_LR3 0
#endif

#ifndef USE_LATEST_POSE
#define USE_LATEST_POSE 1
#endif

#if 1
#define ERR(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__); \
                      fprintf(stderr, __VA_ARGS__); fflush(stderr); } while(0)
#else
#define ERR(...)
#endif

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#define DBG(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...)
#endif

static GtkWidget *placename_dialog;
static GtkWidget *index_entry;

typedef enum _color_mode_t {
  COLOR_MODE_DRAB, COLOR_MODE_LASER, COLOR_MODE_Z,
} color_mode_t;

typedef struct _laser_channel {
  int enabled;
  char name[256]; /* lidar name (from config file) */
  char conf_path[256]; /* config file path */
  double color[3]; /* point cloud color (from config file) */

  Laser_projector *projector;

  BotPtrCircular *scans; /* circular buffer of laser_scan */
} laser_channel;

typedef struct _laser_point2d {
  double pos[2]; 
} laser_point2d;

typedef struct _laser_segment {
  laser_point2d *points;
  //features 
  int index; 
  int is_person; 
  int no_points;
  double width;  
  double jump_dist_pre;
  double jump_dist_suc;
  double sdev;
  double mean_dev_from_median;
  double linearity;
  double circulrity;
  double radius;
  double boundry_length;
  double boundry_regularity;
  double mean_curvature;
  double mean_angular_difference;
  //double mean_speed;// - harder to do 
} laser_segment; 

typedef struct _segment_collection {
  int no_segments;
  laser_segment *segments;
} segment_collection;


typedef struct _RendererSegmentation {
  Renderer renderer;
  EventHandler ehandler;
  Viewer *viewer;
  BotConf *config;
  Carmen3d_trans *ripl_trans;
  lcm_t *lcm;

  segment_collection current_laser_segments;
  long utime; 
  //int current_segment_no; 

  /* user parameters */
  BotGtkParamWidget *pw;
  int param_scan_memory;
  color_mode_t param_color_mode;
  gboolean param_z_buffer;
  gboolean param_big_points;
  gboolean param_spacial_decimate;
  double param_color_mode_z_max_z;
  double param_color_mode_z_min_z;

  gchar *last_save_filename;

  GHashTable *channels_hash; /* hash of laser_channels, key: lidar
   configuration file name, value: pointer to
   laser_channel */
  GPtrArray *channels; /* array of pointers to all laser_channel */
  GList *lcm_hids;
} RendererSegmentation;

static void laser_scan_destroy(void *user, void *p)
{
  if (p) {
    laser_projected_scan *lscan = (laser_projected_scan*) p;
    laser_destroy_projected_scan(lscan);
  }
}

static void initialize_segment_collection(segment_collection *seg_collection){
  seg_collection->no_segments = 0;
  seg_collection->segments = NULL; 
}

static void reset_segment_collection(segment_collection *seg_collection){
  
  if(seg_collection->segments !=NULL){
    for(int i=0; i< seg_collection->no_segments; i++){
      free(seg_collection->segments[i].points);
    }
    //  free(seg_collection->segments);
  }
  seg_collection->no_segments = 0;
}

static void get_statistics(double *x,double *y, int size, double *stddev, 
			   double *mean_dev_from_median, double *radius, 
			   double *residual_sum, double *boundry_length,
			   double *b_length_sdev, double *k_avg, double *avg_angle){
  double x_mean = 0;
  double y_mean = 0; 
  double x_sq_sum = 0;
  double y_sq_sum = 0;

  x_mean = gsl_stats_mean(x,1,size);
  y_mean = gsl_stats_mean(y,1,size);

  double x_var = gsl_stats_variance(x,1,size);
  double y_var = gsl_stats_variance(y,1,size);

  double gsl_var = hypot(x_var, y_var);

  *stddev = pow(gsl_var,0.5);

  //fprintf(stderr, "gsl_variance : %f\n", gsl_var);

  /*for(int i=0; i< size; i++){
    x_sq_sum += x[i]*x[i]; 
    y_sq_sum += y[i]*y[i]; 
    
    //x_mean += x[i];
    //y_mean += y[i];
  }

  //x_mean /=size;
  //y_mean /=size; 

  double x_comp = x_sq_sum / size - pow(x_mean,2);
  double y_comp = y_sq_sum / size - pow(y_mean,2);

  double var = size / (size-1) * (x_comp + y_comp); //this is correct  

  fprintf(stderr, "calculated_variance : %f\n", gsl_var);

  *stddev = pow(var, 0.5);*/ 
  double *x_sorted = calloc(size, sizeof(double));
  double *y_sorted = calloc(size, sizeof(double));

  memcpy(x_sorted, x, size* sizeof(double));
  memcpy(y_sorted, y, size* sizeof(double));

  gsl_sort (x_sorted, 1, size);
  gsl_sort (y_sorted, 1, size);
  
  double x_median = gsl_stats_median_from_sorted_data (x_sorted, 
						       1, size);

  double y_median = gsl_stats_median_from_sorted_data (y_sorted, 
						       1, size);

  free(x_sorted);
  free(y_sorted);

  *mean_dev_from_median = 0;
  
  for(int i=0; i< size; i++){
    *mean_dev_from_median += hypot(x[i] - x_median, y[i] - y_median);
  }
  
  *mean_dev_from_median /=size;

  //circulatrity 
  
  double *a_data = calloc(size * 3, sizeof(double)); 
  double *b_data = calloc(size, sizeof(double)); 

  /*double *a_mul_data = calloc(size*size, sizeof(double)); 
  memset(a_mul_data, 0, size*size*sizeof(double)); 
  */
  for(int i=0; i< size; i++){
    a_data[3*i] = -2 * x[i];
    a_data[3*i+1] = -2 * y[i];
    a_data[3*i+2] = 1;

    b_data[i] = -pow(x[i],2) -pow(y[i],2);
  }
  
  gsl_matrix_view a_mat = gsl_matrix_view_array (a_data, size, 3);
  gsl_matrix_view b_mat = gsl_matrix_view_array (b_data, size, 1);
  gsl_matrix *a_trans_mat = gsl_matrix_alloc(3,size);
  //gsl_matrix_view_array (a_mul_data, size, size);//gsl_matrix_alloc(3,size);//gsl_matrix_view_array (a_data, size, 3);

  gsl_matrix *a_result = gsl_matrix_alloc(3,3);
  gsl_matrix *a_inv_result = gsl_matrix_alloc(3,3);
  gsl_matrix *a_inv_trans_result = gsl_matrix_alloc(3,size);
  
  gsl_matrix *solution = gsl_matrix_alloc(3,1);
  
  gsl_matrix_transpose_memcpy(a_trans_mat, &a_mat.matrix); 

  //gsl_matrix_mul_elements(a_trans_mat,a_mat);  

  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
		  1.0, a_trans_mat, &a_mat.matrix,//->matrix,
		  0.0, a_result);//->matrix);

  int s;
  gsl_permutation * p = gsl_permutation_alloc (3);

  gsl_linalg_LU_decomp (a_result, p, &s);
  gsl_linalg_LU_invert(a_result, p, a_inv_result);

  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
		  1.0, a_inv_result, a_trans_mat,//->matrix,
		  0.0, a_inv_trans_result);

  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
		  1.0, a_inv_trans_result, &b_mat.matrix,//->matrix,
		  0.0, solution);

  gsl_matrix_fprintf(stdout, solution, "%g");

  double x_c = gsl_matrix_get(solution, 0,0);
  double y_c = gsl_matrix_get(solution, 1,0);
  double k = gsl_matrix_get(solution, 2,0);
  *radius = pow(pow(x_c,2) + pow(y_c,2) - k, 0.5); 

  *residual_sum = 0;
  
  for(int i=0; i< size; i++){
    *residual_sum += pow(*radius - pow(pow(x[i] - x_c,2) + pow(y[i] - y_c, 2),0.5),2); 
  }  

  fprintf(stderr, " Radius : %f\n",   *radius);

  //gsl_linalg_cholesky_decomp(a_result); //doesnt work - needs to be positive semi-definite
  //gsl_linalg_cholesky_invert(a_result);

  //gsl_linalg_cholesky_invert(a_trans_mat)
  
  //gsl_matrix_view b = gsl_vector_view_array (b_data, size);

  //gsl_vector *x = gsl_vector_alloc (4);
  free(a_data);
  free(b_data);
  gsl_matrix_free (a_trans_mat);
  gsl_matrix_free (a_result);
  gsl_matrix_free (a_inv_result);
  gsl_matrix_free (a_inv_trans_result);
  gsl_matrix_free (solution);  

  *boundry_length = 0;

  double *b_lengths = calloc(size-1, sizeof(double)); 

  for(int i=1; i< size; i++){
    b_lengths[i-1] = hypot(x[i] - x[i-1], y[i] - y[i-1]);
    *boundry_length += b_lengths[i-1]; 
  }
  
  double b_length_var = gsl_stats_variance(b_lengths,1,size-1);
  *b_length_sdev = pow(b_length_var,0.5);

  free(b_lengths);

  *k_avg = 0;
  *avg_angle = 0;
  for(int i=2; i< size; i++){
    double xy1[2] = {x[i] - x[i-2], y[i] - y[i-2]};
    double xy2[2] = {x[i-1] - x[i-2], y[i-1] - y[i-2]};
    double xy3[2] = {x[i] - x[i-1], y[i] - y[i-1]};

    double dtheta = atan2(xy3[1], xy3[0]) - atan2(xy2[1], xy2[0]); 

    while(dtheta > M_PI){
      dtheta -=2*M_PI; 
    }

    while(dtheta < -M_PI){
      dtheta +=2*M_PI; 
    }

    *avg_angle +=dtheta; 

    double d1 = hypot(x[i]-x[i-1], y[i]-y[i-1]);
    double d2 = hypot(x[i]-x[i-2], y[i]-y[i-2]);
    double d3 = hypot(x[i-2]-x[i-1], y[i-2]-y[i-1]);

    double area = fabs(xy1[0]*xy2[1] - xy2[0] * xy1[1]) / 2; 
    *k_avg += 4 * area / (d1 * d2 * d3);
  }
  *avg_angle /=(size -2);
  *k_avg /=(size-2);

}

static int calculate_features(laser_segment *curr_seg){
  int size = curr_seg->no_points; 
  double width = hypot(curr_seg->points[0].pos[0]- curr_seg->points[size-1].pos[0], curr_seg->points[0].pos[1]- curr_seg->points[size-1].pos[1]);
  if(width > 1.5){//too large - we ignore these segments 
    //fprintf(stderr,"Ignoring - Too large\n");
    return -1;
  }
  if(size <= 2){
    //fprintf(stderr,"Ignoring : Too few points\n");
    return -1;
  }

  double *x = calloc(size, sizeof(double));
  double *y = calloc(size, sizeof(double));
  double c0, c1, cov00, cov01, cov11, sumsq; //c0, c1, 
  
  for(int i=0;i< size; i++){
    x[i] = curr_seg->points[i].pos[0];
    y[i] = curr_seg->points[i].pos[1];
  }
  
  gsl_fit_linear (x,1, y,1, size, &c0, &c1, &cov00, &cov01, &cov11, &sumsq);
  //fprintf(stderr,"C0 : %f C1: %f\n", c0, c1);
  
  //gsl_sort(xy, 2, size);

  double stddev = 0;
  double mean_dev_from_median = 0;
  double radius = 0;
  double circularity = 0;
  double boundry_length = 0;
  double b_length_sdev = 0;
  double k_avg = 0;
  double avg_angle = 0;
  get_statistics(x,y,size, &stddev, &mean_dev_from_median, &radius, &circularity, &boundry_length, &b_length_sdev, &k_avg, &avg_angle);

  curr_seg->sdev = stddev;
  curr_seg->mean_dev_from_median = mean_dev_from_median; 
  curr_seg->width = width; 
  curr_seg->linearity = sumsq;
  curr_seg->circulrity = circularity;
  curr_seg->radius = radius; 
  curr_seg->boundry_length = boundry_length;
  curr_seg->boundry_regularity = b_length_sdev;  
  curr_seg->mean_curvature = k_avg;
  curr_seg->mean_angular_difference = avg_angle;
  curr_seg->is_person = 0;

  free(x);
  free(y);  
  fprintf(stderr,"Size : %d Width : %f, Std Dev : %f Mean Dev from median : %f Sum Sq : %f\n",size, width, stddev, mean_dev_from_median, sumsq);
  fprintf(stderr,"Radius : %f, Circularity : %f Boundry Length : %f B length dev : %f k avg : %f k_avg_angle :%f\n", radius, circularity, boundry_length, b_length_sdev, k_avg, avg_angle);
}

static void add_to_segment_collection(segment_collection *seg_collection, laser_segment *curr_seg){
  //print out the statistcs 
  /*for(int i=0;i< curr_seg->no_points; i++){
    fprintf(stderr,"\t%d : %f,%f\n", i, curr_seg->points[i].pos[0], curr_seg->points[i].pos[1]);
    }*/

  int to_add = calculate_features(curr_seg);

  if(to_add ==-1){    
    return;
  }

  fprintf(stderr, "Added new Segment : %d Size : %d, Pre : %f Suc : %f\n", seg_collection->no_segments, curr_seg->no_points, 
	  curr_seg->jump_dist_pre, curr_seg->jump_dist_suc);

  seg_collection->no_segments++;
  //fprintf(stderr,"Trying to add segment\n");
  seg_collection->segments = (laser_segment *)realloc(seg_collection->segments, 
						      sizeof(laser_segment)*seg_collection->no_segments);

  //fprintf(stderr,"Reallocated\n");
  memcpy(&seg_collection->segments[seg_collection->no_segments-1], curr_seg,sizeof(laser_segment));
  seg_collection->segments[seg_collection->no_segments-1].points = (laser_point2d *) malloc(curr_seg->no_points * sizeof(laser_point2d));
  memcpy(seg_collection->segments[seg_collection->no_segments-1].points, curr_seg->points, curr_seg->no_points * sizeof(laser_point2d));
  //fprintf(stderr,"Coppied\n");
  seg_collection->segments[seg_collection->no_segments-1].index = seg_collection->no_segments - 1; 
  //fprintf(stderr, "Added new Segment : %d\n", seg_collection->no_segments);
}

static void save_segment_collection(segment_collection *seg_collection){
  //seg_collection->no_segments = 0;
  //seg_collection->segments = NULL; 
}

static int do_segmentation(RendererSegmentation *self){
  //reset the segments 
  reset_segment_collection(&self->current_laser_segments);

  for (int chan_idx = 0; chan_idx < self->channels->len; chan_idx++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, chan_idx);
    if (!lchan->enabled)
      continue;
    //        printf("%s has %d points.\n", lchan->name, lchan->scans->size);
    int scan_count = MIN(self->param_scan_memory, bot_ptr_circular_size(lchan->scans));
    for (int scan_idx = 0; scan_idx < scan_count; scan_idx++) {
      laser_projected_scan *lscan = bot_ptr_circular_index(lchan->scans, scan_idx);
      if (!lscan->projection_status){
	laser_update_projected_scan(lchan->projector,lscan,"local");
	//laser_update_projected_scan(lchan->projector,lscan,"body");
      }

      int i_back = 0;
      double distance = 0.0;

      laser_segment curr_seg; 
      memset(&curr_seg, 0, sizeof(laser_segment));
      
      laser_point2d *seg_points = malloc(sizeof(laser_point2d) * lscan->npoints);

      //set the first point as the first element in the first segment 
      seg_points[0].pos[0] = lscan->points[0].x;
      seg_points[0].pos[1] = lscan->points[0].y;
      curr_seg.no_points = 1;
      
      curr_seg.jump_dist_pre = 100;//not sure what to set for the start and end segments - maybe we can just ignore them 
      
      double max_seg_point_gap =  0.1;

      for (int i = 1; i < lscan->npoints; i++) {
        if (lscan->invalidPoints[i]){
	  //fprintf(stderr," %f,%f\n",lscan->points[i].x, lscan->points[i].y);
	  //fprintf(stderr,"Invalid Point\n");
          continue; /* invalid point */  	  
	}

	distance = hypot(lscan->points[i].x -lscan->points[i_back].x,
    		     lscan->points[i].y -lscan->points[i_back].y );

	//fprintf(stderr,"%d %d : %f\n", i, i_back, distance);

	if(distance < max_seg_point_gap && i <= lscan->npoints-1 ){ 
	  i_back++;
	  if(curr_seg.no_points==0){//this has been reset - calculate the break dist 
	    fprintf(stderr,"Errorrr\n");
	  }

	  seg_points[curr_seg.no_points].pos[0] = lscan->points[i].x;
	  seg_points[curr_seg.no_points].pos[1] = lscan->points[i].y;
	  curr_seg.no_points++;

	  if(i==lscan->npoints-1){
	    fprintf(stderr,"Last Point \n");
	    curr_seg.jump_dist_suc = 100; //not sure what to set this to 
	    curr_seg.points = malloc(sizeof(laser_point2d) * curr_seg.no_points);
	    memcpy(curr_seg.points,seg_points, sizeof(laser_point2d) * curr_seg.no_points);
	    
	    //send this off to calculate stats 
	    //add this as a segment 
	    add_to_segment_collection(&self->current_laser_segments,&curr_seg);
	    free(curr_seg.points);
	    //***********************
	  }
	}
	else{
	  int outlier_found = 0;
	  double lookahead_dist = 0;
	  //added handling of outliers 
	  for(int j = fmin(i+1, lscan->npoints-1); j < fmin(i+3, lscan->npoints-1);j++){
	    lookahead_dist = hypot(lscan->points[j].x - lscan->points[i_back].x,
			     lscan->points[j].y - lscan->points[i_back].y );
	    if(lookahead_dist < max_seg_point_gap){
	      seg_points[curr_seg.no_points].pos[0] = lscan->points[j].x;
	      seg_points[curr_seg.no_points].pos[1] = lscan->points[j].y;
	      curr_seg.no_points++;
	      outlier_found = 1;
	      i_back = j;
	      i = i_back;
	      break;
	    }
	  }
	  if(!outlier_found){//we have found an end of segment 
	    curr_seg.jump_dist_suc = distance;
	    
	    //add the as a segment 
	    curr_seg.points = malloc(sizeof(laser_point2d) * curr_seg.no_points);
	    memcpy(curr_seg.points,seg_points, sizeof(laser_point2d) * curr_seg.no_points);
	    
	    //send this off to calculate stats 
	    //add this as a segment 
	    add_to_segment_collection(&self->current_laser_segments,&curr_seg);
	    free(curr_seg.points);
	    
	    curr_seg.jump_dist_pre = distance; 
	    curr_seg.no_points = 0;
	    seg_points[curr_seg.no_points].pos[0] = lscan->points[i].x;
	    seg_points[curr_seg.no_points].pos[1] = lscan->points[i].y;
	    curr_seg.no_points++;
	    i_back = i;
	  //reset the segments we have and set the 
	  }
	}
      }
      free(seg_points);
    }
  }
}

static
void save_person_index(GtkWidget *button __attribute__ ((unused)),
		      gpointer user_data __attribute__ ((unused)))
{
  RendererSegmentation *self = (RendererSegmentation *) user_data;
  char index_char[100];
  strcpy(index_char, gtk_entry_get_text(GTK_ENTRY(index_entry)));
  
  int index = atoi(index_char);
  
  fprintf(stderr, "Index : %d\n", index);

  fprintf(stderr,"Done :%d\n", self->current_laser_segments.no_segments);
  if(self->current_laser_segments.no_segments> index){
    //set is_person to 1
    self->current_laser_segments.segments[index].is_person = 1;
  }
  //this is the only valid index - save to file 
  char filename[100]; 
  sprintf(filename, "logs/segments_%ld.dat",self->utime);
  fprintf(stderr,"Saving to individual_file : %s\n", filename);

  FILE *fp;
  fp=fopen(filename, "w");

  FILE *fp_full;
  char filename_full[100]; 
  sprintf(filename_full, "logs/training_data.dat");
  fp_full=fopen(filename_full, "a");
  
  if(self->current_laser_segments.no_segments >0){
    for(int i=0; i < self->current_laser_segments.no_segments; i++){
      laser_segment *lc = &self->current_laser_segments.segments[i];
      int is_person = 2;
      if(lc->is_person==1){
	is_person =1;
      }
      fprintf(fp,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
	      lc->no_points, lc->width, lc->jump_dist_pre, 
	      lc->jump_dist_suc,lc->sdev, lc->mean_dev_from_median, 
	      lc->linearity, lc->circulrity, 
	      lc->radius, lc->boundry_length, 
	      lc->boundry_regularity, lc->mean_curvature,
	      lc->mean_angular_difference, is_person);    
      fprintf(fp_full,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
	      lc->no_points, lc->width, lc->jump_dist_pre, 
	      lc->jump_dist_suc,lc->sdev, lc->mean_dev_from_median, 
	      lc->linearity, lc->circulrity, 
	      lc->radius, lc->boundry_length, 
	      lc->boundry_regularity, lc->mean_curvature,
	      lc->mean_angular_difference, is_person); 
      fprintf(stderr,"%d:%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",lc->index,
	      lc->no_points, lc->width, lc->jump_dist_pre, 
	      lc->jump_dist_suc,lc->sdev, lc->mean_dev_from_median, 
	      lc->linearity, lc->circulrity, 
	      lc->radius, lc->boundry_length, 
	      lc->boundry_regularity, lc->mean_curvature,
	      lc->mean_angular_difference, is_person);   
    }
  }
  fclose(fp);
  fclose(fp_full);  
  gtk_widget_destroy(placename_dialog);
  fprintf(stderr,"Done\n");
}

static
void add_person_index(GtkWidget *button __attribute__ ((unused)),
		      gpointer user_data __attribute__ ((unused)))
{
  RendererSegmentation *self = (RendererSegmentation *) user_data;
  char index_char[100];
  strcpy(index_char, gtk_entry_get_text(GTK_ENTRY(index_entry)));
  
  int index = atoi(index_char);
  
  fprintf(stderr, "Index : %d\n", index);

  fprintf(stderr,"Done :%d\n", self->current_laser_segments.no_segments);
  if(self->current_laser_segments.no_segments> index){
    //set is_person to 1
    self->current_laser_segments.segments[index].is_person = 1;
  }
  //gtk_widget_destroy(placename_dialog);
}

void label_person_index(RendererSegmentation *self)
{  
  static GtkWidget *name_label, *x_label, *y_label, *theta_label;
  static GtkWidget *theta_entry, *x_std_entry, 
    *y_std_entry, *theta_std_entry;
  int edit_place_id;

  GtkWidget *hbox, *label, *button;
  char buffer[10];

  placename_dialog = gtk_dialog_new();
  hbox = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start (GTK_BOX (GTK_DIALOG (placename_dialog)->vbox),
		      hbox, TRUE, TRUE, 0);
  name_label = gtk_label_new("Person indexes:");
  gtk_box_pack_start (GTK_BOX (hbox), name_label, TRUE, TRUE, 0);
  index_entry = gtk_entry_new_with_max_length(21);
  gtk_widget_set_usize(index_entry, 90, 20);
  gtk_box_pack_start (GTK_BOX(hbox), index_entry, TRUE, TRUE, 0);

  button = gtk_button_new_with_label("Save Points");
  gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);
  gtk_signal_connect(GTK_OBJECT(button), "clicked", 
			 (GtkSignalFunc)save_person_index, self);
  
  button = gtk_button_new_with_label("Add Index");
  gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);
  
  gtk_signal_connect(GTK_OBJECT(button), "clicked", 
			 (GtkSignalFunc)add_person_index, self);
  
  button = gtk_button_new_with_label("Cancel");
  gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);		
  
  gtk_signal_connect_object(GTK_OBJECT(button),
			    "clicked", (GtkSignalFunc)gtk_widget_destroy, 
			    (gpointer)placename_dialog);	

  gtk_widget_show_all(placename_dialog);
}

static int 
mouse_press (Viewer *viewer, EventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererSegmentation *self = (RendererSegmentation*) ehandler->user;

  fprintf(stderr, "Mouse Press : %f,%f\n", ray_start[0], ray_start[1]);
  
  /*do_segmentation(self);*/

  viewer_request_redraw(self->viewer);
  return 1;
}

static void renderer_laser_destroy(Renderer *renderer)
{
  if (!renderer)
    return;

  RendererSegmentation *self = (RendererSegmentation*) renderer->user;
  if (!self)
    return;

  /* stop listening to lcm */
  if (self->lcm) {
    for (GList *iter = self->lcm_hids; iter; iter = iter->next) {
      bot_core_planar_lidar_t_subscription_t *hid = (bot_core_planar_lidar_t_subscription_t *) iter->data;
      bot_core_planar_lidar_t_unsubscribe(self->lcm, hid);
    }
  }

  /* free channels hash table and array */
  if (self->channels) {
    for (int i = 0; i < self->channels->len; i++) {
      laser_channel *lchan = g_ptr_array_index(self->channels, i);
      if (lchan->scans)
        bot_ptr_circular_destroy(lchan->scans);
      if (lchan->projector)
        laser_projector_destroy(lchan->projector);
      free(lchan);
    }
    g_ptr_array_free(self->channels, FALSE);
  }

  if (self->channels_hash)
    g_hash_table_destroy(self->channels_hash);

  if (self->last_save_filename)
    g_free(self->last_save_filename);

  if (self->ripl_trans)
    globals_release_ripl_trans(self->ripl_trans);
  if (self->lcm)
    globals_release_lcm(self->lcm);
  if (self->config)
    globals_release_config(self->config);

  free(self);
}

static void renderer_laser_draw(Viewer *viewer, Renderer *renderer)
{
  RendererSegmentation *self = (RendererSegmentation*) renderer->user;
  g_assert(self);

  glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
  if (self->param_z_buffer) {
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
  }

  if (self->param_big_points)
    glPointSize(8.0f);
  else
    glPointSize(2.0f);

  if(self->current_laser_segments.no_segments >0){//draw the segments 
    glColor3d(1.0,.0,.0);//lchan->color[0], lchan->color[1], lchan->color[2]);
    for(int i=0; i < self->current_laser_segments.no_segments; i++){
      laser_segment *lc = &self->current_laser_segments.segments[i];
      char label[1042];

      /*sprintf(label,"Index:%d\n points:%d width:%f,\n sdev:%f dev_from_median:%f\n linearity:%fradius:%f, circularity:%f\n b_length:%f b_dev:%f\n k_avg:%f k_avg_angle:%f\n",lc->index,
	      lc->no_points, lc->width, lc->sdev, lc->mean_dev_from_median, lc->linearity, 
	      lc->radius, lc->circulrity, lc->boundry_length, lc->boundry_regularity, lc->mean_curvature,
	      lc->mean_angular_difference);*/

      sprintf(label,"Index:%d", lc->index);
     
      double textpos[3] = {lc->points[0].pos[0], lc->points[0].pos[1], 0};
      bot_gl_draw_text(textpos, GLUT_BITMAP_HELVETICA_12, label, BOT_GL_DRAW_TEXT_DROP_SHADOW);
    }    
  }

  glBegin(GL_POINTS);

  /* get local position in order to height-color points */
  color_mode_t color_mode = self->param_color_mode;
  double pos[3]={0}, z_norm_scale=0;
  if (color_mode == COLOR_MODE_Z) {
    if (!ripl_trans_vehicle_pos_local(self->ripl_trans, pos))
      color_mode = COLOR_MODE_DRAB; /* no position */
    else
      z_norm_scale = 1 / (self->param_color_mode_z_max_z - self->param_color_mode_z_min_z);
  }

  if(self->current_laser_segments.no_segments >0){//draw the segments 
    glColor3d(1.0,.0,.0);//lchan->color[0], lchan->color[1], lchan->color[2]);
    for(int i=0; i < self->current_laser_segments.no_segments; i++){
      laser_segment *ls = &self->current_laser_segments.segments[i];
      //fprintf(stderr,"Segment : %d No : %d\n", i, ls->no_points);
      //char label[256];
      //sprintf(label, "Test");
      //double textpos[3] = {ls->points[0].pos[0], ls->points[0].pos[1], 0};
      //bot_gl_draw_text(textpos, GLUT_BITMAP_HELVETICA_12, "label", BOT_GL_DRAW_TEXT_DROP_SHADOW);
      
      for(int j=0;j < ls->no_points; j++){
	double pt[3] = {ls->points[j].pos[0], ls->points[j].pos[1], 0};
	//	fprintf(stderr,"\t[%f,%f]\n", pt[0], pt[1]);
	glVertex3dv(pt);
      }
    }
  }
  
  
  for (int chan_idx = 0; chan_idx < self->channels->len; chan_idx++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, chan_idx);
    if (!lchan->enabled)
      continue;
    //        printf("%s has %d points.\n", lchan->name, lchan->scans->size);
    int scan_count = MIN(self->param_scan_memory, bot_ptr_circular_size(lchan->scans));
    for (int scan_idx = 0; scan_idx < scan_count; scan_idx++) {
      laser_projected_scan *lscan = bot_ptr_circular_index(lchan->scans, scan_idx);
      if (!lscan->projection_status){
	laser_update_projected_scan(lchan->projector,lscan,"local");
	//laser_update_projected_scan(lchan->projector,lscan,"body");
      }


      for (int i = 0; i < lscan->npoints; i++) {
        if (lscan->invalidPoints[i])
          continue; // invalid point 

        switch (color_mode) {
        case COLOR_MODE_LASER:
          glColor3d(lchan->color[0], lchan->color[1], lchan->color[2]);
          break;
        case COLOR_MODE_Z:
          {
            double z = lscan->points[i].z;
            double z_norm = (z - self->param_color_mode_z_min_z) * z_norm_scale;
            glColor3fv(color_util_jet(z_norm));
          }
          break;
        default: // COLOR_MODE_DRAB 
          glColor3d(0.3, 0.3, 0.3);
          break;
        }
	
	//fprintf(stderr,"%d : %f,%f,%f\n", i, point3d_as_array(&lscan->points[i])[0], point3d_as_array(&lscan->points[i])[1], point3d_as_array(&lscan->points[i])[2]);
        glVertex3dv(point3d_as_array(&lscan->points[i]));
      }
    }
  }

  glEnd();
  glPopAttrib();
}

static void on_laser(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_planar_lidar_t *msg, void *user)
{
  RendererSegmentation *self = (RendererSegmentation*) user;
  g_assert(self);
  self->utime = msg->utime;

  /* get the laser channel object based on channel name */
  laser_channel *lchan = g_hash_table_lookup(self->channels_hash, channel);
  if (lchan == NULL) {
    /* allocate and initialize laser channel structure */
    lchan = (laser_channel*) calloc(1, sizeof(laser_channel));
    g_assert(lchan);
    lchan->enabled = 1;
    lchan->projector = laser_projector_new(channel, 1);
    strcpy(lchan->name, channel);
    arconf_get_planar_lidar_config_path(NULL, lchan->name, lchan->conf_path, sizeof(lchan->conf_path));

    char color_key[256];
    sprintf(color_key, "%s.viewer_color", lchan->conf_path);

   
    //fprintf(stderr,"Laser Flipped %d",flipped);
    int color_size = bot_conf_get_double_array(self->config, color_key, lchan->color, 3);
    if (color_size != 3) {
      ERR("Error: Missing or funny color for planar LIDAR "
          "configuration key: '%s'\n", color_key);
      lchan->color[0] = 1;
      lchan->color[1] = 1;
      lchan->color[2] = 1;
    }

    lchan->scans = bot_ptr_circular_new(MAX_SCAN_MEMORY, laser_scan_destroy, NULL);
    g_assert(lchan->scans);

    /* add laser channel to hash table and array */
    g_hash_table_insert(self->channels_hash, lchan->name, lchan);
    g_ptr_array_add(self->channels, lchan);

    /* add check box */
    if (self->viewer)
      bot_gtk_param_widget_add_booleans(self->pw, 0, lchan->name, lchan->enabled, NULL);
  }

  /* TODO: Optimization - allocate space for local points from a
   circular buffer instead of calling calloc for each scan */

  laser_projected_scan *lscan = laser_create_projected_scan_from_planar_lidar(lchan->projector, msg, "local");//, ang_offset);
  //laser_projected_scan *lscan = laser_create_projected_scan_from_planar_lidar(lchan->projector, msg, "body");
  if (lscan == NULL)
    return; //probably didn't have a pose message yet...
  if (bot_ptr_circular_size(lchan->scans) > 0) {
    laser_projected_scan *last_scan = bot_ptr_circular_index(lchan->scans, 0);

    /* check for a large time difference between scans (typical when
     jumping around an LCM log file) */
    gboolean time_jump = FALSE;
    int64_t dt = msg->utime - last_scan->utime;
    if (dt < -OLD_HISTORY_THRESHOLD || dt > OLD_HISTORY_THRESHOLD)
      time_jump = TRUE;

    /* spacial decimation */
    gboolean stationary = FALSE;
    if (self->param_spacial_decimate && self->param_scan_memory > 10) {
      double d[3];
      bot_vector_subtract_3d(point3d_as_array(&lscan->origin), point3d_as_array(&last_scan->origin), d);
      double dist = bot_vector_magnitude_3d(d);
      if (dist < SPACIAL_DECIMATION_LIMIT) {
        stationary = TRUE;
      }
    }

    if (stationary) {
      laser_scan_destroy(NULL, lscan);
      return;
    }
  }

  bot_ptr_circular_add(lchan->scans, lscan);

  if (self->viewer)
    viewer_request_redraw(self->viewer);

  return;
}

static void on_save_index_button(GtkWidget *button, RendererSegmentation *self)
{
  if (!self->viewer)
    return;
  label_person_index(self);
}

static void on_display_button(GtkWidget *button, RendererSegmentation *self)
{
  if (!self->viewer)
    return;

  do_segmentation(self);

  viewer_request_redraw(self->viewer);

  label_person_index(self);
}

static int save_points_to_file(RendererSegmentation *self, FILE *file)
{
//  /* first line: number of columns */
//  fprintf(file, "%%%% %d\n", 10);
//  /* second line: column titles */
//  fprintf(file, "%%%% POINT_X POINT_Y POINT_Z ORIGIN_X ORIGIN_Y ORIGIN_Z"
//    " POINT_STATUS LASER_NUMER SCAN_NUMER POINT_NUMBER\n");
//
  int count = 0;
//  for (int chan_idx = 0; chan_idx < self->channels->len; chan_idx++) {
//    laser_channel *lchan = g_ptr_array_index(self->channels, chan_idx);
//
//    int scan_count = MIN(self->param_scan_memory, bot_ptr_circular_size(lchan->scans));
//    for (int scan_idx = 0; scan_idx < scan_count; scan_idx++) {
//      laser_projected_scan *lscan = bot_ptr_circular_index(lchan->scans, scan_idx);
//
//      for (int i = 0; i < lscan->npoints; i++) {
//        fprintf(file, "%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %d %d %d "
//          "%d\n", lscan->points[i].x, lscan->points[i].y, lscan->points[i].z, lscan->origin.x, lscan->origin.y,
//            lscan->origin.z, lscan->invalidPoints[i], chan_idx, scan_idx, i);
//        count++;
//      }
//    }
//  }
  return count;
}

static void on_save_button(GtkWidget *button, RendererSegmentation *self)
{
  GtkWidget *dialog;
  dialog = gtk_file_chooser_dialog_new("Save Data File", NULL, GTK_FILE_CHOOSER_ACTION_SAVE, GTK_STOCK_CANCEL,
      GTK_RESPONSE_CANCEL, GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT, NULL);
  g_assert(dialog);
  gtk_file_chooser_set_do_overwrite_confirmation(GTK_FILE_CHOOSER(dialog), TRUE);
  if (self->last_save_filename)
    gtk_file_chooser_set_filename(GTK_FILE_CHOOSER(dialog), self->last_save_filename);

  if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
    char *filename;
    filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
    if (NULL != filename) {
      FILE *file = fopen(filename, "w");
      if (NULL != file) {
        /* store selected file into self->last_save_filename */
        if (self->last_save_filename)
          g_free(self->last_save_filename);
        self->last_save_filename = g_strdup(filename);

        int count = save_points_to_file(self, file);

        DBG("Wrote %d points to file '%s'.\n", count, filename);
        fclose(file);
      }
      else {
        ERR("Error: Failed to open file: '%s', error is: '%s'\n",
            filename, g_strerror(errno));
      }
      g_free(filename);
    }
  }
  gtk_widget_destroy(dialog);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererSegmentation *self = (RendererSegmentation*) user;

  if (!self->viewer)
    return;

  self->param_scan_memory = bot_gtk_param_widget_get_int(self->pw, PARAM_SCAN_MEMORY);
  self->param_color_mode = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);
  self->param_z_buffer = bot_gtk_param_widget_get_bool(self->pw, PARAM_Z_BUFFER);
  self->param_big_points = bot_gtk_param_widget_get_bool(self->pw, PARAM_BIG_POINTS);
  self->param_spacial_decimate = bot_gtk_param_widget_get_bool(self->pw, PARAM_SPATIAL_DECIMATE);
  self->param_color_mode_z_max_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);
  self->param_color_mode_z_min_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);

  for (int i = 0; i < self->channels->len; i++) {
    laser_channel *lchan = g_ptr_array_index(self->channels, i);
    lchan->enabled = bot_gtk_param_widget_get_bool(pw, lchan->name);
  }

  viewer_request_redraw(self->viewer);
}

static void on_load_preferences(Viewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  RendererSegmentation *self = (RendererSegmentation*) user;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(Viewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  RendererSegmentation *self = (RendererSegmentation*) user;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

Renderer* renderer_segmentation_new(Viewer *viewer, int render_priority)
{
  RendererSegmentation *self = (RendererSegmentation*) calloc(1, sizeof(RendererSegmentation));
  g_assert(self);

  self->viewer = viewer;

  Renderer *renderer = &self->renderer;
  renderer->draw = renderer_laser_draw;
  renderer->destroy = renderer_laser_destroy;
  renderer->user = self;
  renderer->name = RENDERER_NAME;
  renderer->enabled = 1;

  EventHandler *ehandler = &self->ehandler;
  ehandler->name = (char*) RENDERER_NAME;
  ehandler->enabled = 1;
  ehandler->pick_query = NULL;
  ehandler->key_press = NULL;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = mouse_press;
  ehandler->mouse_release = NULL;
  ehandler->mouse_motion = NULL;
  ehandler->user = self;

  viewer_add_event_handler(viewer, &self->ehandler, render_priority);

  // Get global variables.
  self->config = globals_get_config();
  if (!self->config) {
    ERR("Error: setup_renderer_laser() failed to get global config "
        "object\n");
    renderer_laser_destroy(renderer);
    return NULL;
  }
  self->ripl_trans = globals_get_ripl_trans();
  self->lcm = globals_get_lcm_full(NULL, 1);
  if (!self->lcm) {
    ERR("Error: setup_renderer_laser() failed to get global lcm "
        "object\n");
    renderer_laser_destroy(renderer);
    return NULL;
  }

  initialize_segment_collection(&self->current_laser_segments);
  
  self->param_scan_memory = 50;
  self->param_color_mode = COLOR_MODE_LASER;
  self->param_z_buffer = FALSE;
  self->param_big_points = FALSE;
  self->param_spacial_decimate = FALSE;
  self->param_color_mode_z_max_z = COLOR_MODE_Z_MAX_Z;
  self->param_color_mode_z_min_z = COLOR_MODE_Z_MIN_Z;

  if (viewer) {
    /* setup parameter widget */
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    renderer->widget = gtk_vbox_new(FALSE, 0);
    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int(self->pw, PARAM_SCAN_MEMORY, BOT_GTK_PARAM_WIDGET_SLIDER, 1, MAX_SCAN_MEMORY, 1,
        self->param_scan_memory);
    bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, self->param_color_mode,
        "Segmentation", COLOR_MODE_LASER, "Drab", COLOR_MODE_DRAB, "Height", COLOR_MODE_Z, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_Z_BUFFER, self->param_z_buffer, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_BIG_POINTS, self->param_big_points, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, 0, PARAM_SPATIAL_DECIMATE, self->param_spacial_decimate, NULL);
    bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SLIDER,
        COLOR_MODE_Z_MIN_Z, COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_color_mode_z_max_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SLIDER,
        COLOR_MODE_Z_MIN_Z, COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_color_mode_z_min_z);
    GtkWidget *clear_button = gtk_button_new_with_label("Display Features");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_display_button), self);
    GtkWidget *save_index_button = gtk_button_new_with_label("Tag Person");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_index_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_index_button), "clicked", G_CALLBACK(on_save_index_button), self);    
    GtkWidget *save_button = gtk_button_new_with_label("Save To Points File");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_button), "clicked", G_CALLBACK(on_save_button), self);
    gtk_widget_show_all(renderer->widget);

    /* setup signal callbacks */
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
    g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
  }

  // iterate through planar lidars, subscribing to thier LCM and
  // initializing the channels' hash table and array
  self->channels_hash = g_hash_table_new(g_str_hash, g_str_equal);
  self->channels = g_ptr_array_new();
  bot_core_planar_lidar_t_subscription_t *hid;

  char **planar_lidar_names = arconf_get_all_planar_lidar_names(self->config);
  if (planar_lidar_names) {
    for (int pind = 0; planar_lidar_names[pind] != NULL; pind++) {
      //      hid = bot_core_planar_lidar_t_subscribe(self->lcm, planar_lidar_names[pind], on_laser, self);
      hid = bot_core_planar_lidar_t_subscribe(self->lcm, "TOP_LASER", on_laser, self);
      self->lcm_hids = g_list_append(self->lcm_hids, hid);
    }
    g_strfreev(planar_lidar_names);
  }
  else {
    fprintf(stderr, "["__FILE__":%d] Error: Could not"
    " get lidar names.\n", __LINE__);
  }
  //just subscribe to the LASER_CHANNEL
  //  hid = bot_core_planar_lidar_t_subscribe(self->lcm, LASER_CHANNEL, on_laser, self);
  //  self->lcm_hids = g_list_append(self->lcm_hids, hid);

  return &self->renderer;
}

void setup_renderer_segmentation(Viewer *viewer, int priority)
{
  fprintf(stderr,"Called\n");
  Renderer *renderer = renderer_segmentation_new(viewer, priority);
  if (viewer && renderer) {
    viewer_add_renderer(viewer, renderer, priority);
  }
}

