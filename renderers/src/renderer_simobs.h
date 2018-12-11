#ifndef SIMOBS_RENDERER_H_
#define SIMOBS_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_simobs(BotViewer *viewer, int priority, lcm_t *_lcm, BotParam *param);

#ifdef __cplusplus
}
#endif

#endif
