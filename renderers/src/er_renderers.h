#ifndef ER_RENDERERS_H_
#define ER_RENDERERS_H_
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif


void setup_renderer_occupancy_map (BotViewer *viewer, int priority, BotParam * param);

void grid_add_renderer_to_viewer(BotViewer *viewer, int render_priority);

void localize_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm);
void navigator_plan_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm, BotFrames *frames);
void setup_renderer_tracks(BotViewer *viewer, int priority, lcm_t *_lcm, BotParam * param);
void add_husky_model_renderer_to_viewer(BotViewer *viewer, int render_priority, BotParam * param, BotFrames * frames);
void setup_renderer_simobs(BotViewer *viewer, int priority, lcm_t *lcm, BotParam *param);
void setup_renderer_rrtstar (BotViewer *viewer, int priority, lcm_t *_lcm);
void setup_renderer_gridmap (BotViewer *viewer, int priority, lcm_t *_lcm, BotParam * param);
void setup_renderer_host_status (BotViewer *viewer, int priority);
void renderer_sensor_status_new (BotViewer *viewer);
void setup_renderer_robot_status (BotViewer *viewer, BotParam *param, int priority);
void setup_renderer_robot_commands (BotViewer *viewer, int priority);
#ifdef __cplusplus
}
#endif

#endif
