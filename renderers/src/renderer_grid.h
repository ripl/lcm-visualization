#ifndef GRID_RENDERER_H_
#define GRID_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_grid_renderer(BotViewer *viewer, int render_priority);


#ifdef __cplusplus
}
#endif

#endif
