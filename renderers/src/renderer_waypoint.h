#ifndef WAYPOINT_RENDERER_H_
#define WAYPOINT_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_waypoint (BotViewer *viewer, int render_priority, lcm_t * lcm);

#ifdef __cplusplus
}
#endif

#endif
