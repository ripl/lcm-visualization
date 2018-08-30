#ifndef NAVIGATOR_PLAN_RENDERER_H_
#define NAVIGATOR_PLAN_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_navigator_plan (BotViewer *viewer, int priority, lcm_t *_lcm, BotParam * param);

#ifdef __cplusplus
}
#endif

#endif
