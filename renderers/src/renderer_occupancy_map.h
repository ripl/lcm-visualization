#ifndef OCCUPANCY_MAP_RENDERER_H_
#define OCCUPANCY_MAP_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_occupancy_map (BotViewer *viewer, int priority, BotParam * param);

#ifdef __cplusplus
}
#endif

#endif
