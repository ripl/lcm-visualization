#ifndef ER_RENDERERS_H_
#define ER_RENDERERS_H_
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void quad_waypoint_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm);
void add_quad_model_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param, BotFrames * frames);

void setup_renderer_occupancy_map (BotViewer *viewer, int priority, BotParam * param);

void grid_add_renderer_to_viewer(BotViewer *viewer, int render_priority);

void localize_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm);
void verify_check_gridmap_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm);
void navigator_plan_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm);
void setup_renderer_tracks(BotViewer *viewer, int priority, lcm_t *_lcm, BotParam * param);
void add_husky_model_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param, BotFrames * frames);
void setup_renderer_simobs(BotViewer *viewer, int priority, lcm_t *lcm, BotParam *param);
void setup_renderer_rrtstar (BotViewer *viewer, int priority, lcm_t *_lcm);
void setup_renderer_gridmap (BotViewer *viewer, int priority, lcm_t *_lcm, BotParam * param);
void add_person_model_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param, BotFrames * frames, lcm_t *lcm);
void setup_renderer_host_status (BotViewer *viewer, int priority);
void renderer_sensor_status_new (BotViewer *viewer);
void setup_renderer_robot_status (BotViewer *viewer, BotParam *param, int priority);
void setup_renderer_robot_commands (BotViewer *viewer, int priority);
//void setup_renderer_arm_trace_circle (BotViewer *viewer, int priority);
void setup_renderer_pcl (BotViewer *viewer, int priority, BotParam * param);
void add_robot_arm_sensor_renderer_to_viewer(BotViewer *viewer, BotParam *param, int render_priority);
void setup_renderer_manual_calib(BotViewer *viewer, int render_priority);

void setup_renderer_topological_graph (BotViewer *viewer, int priority, BotParam * param);

    //void setup_renderer_annotation(BotViewer *viewer, int priority);
    //void log_annotation_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param);
void setup_renderer_vision_lcmgl (BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param);
    //void setup_renderer_place_classification(BotViewer *viewer, int render_priority, BotParam * param);

#ifdef __cplusplus
}
#endif

#endif
