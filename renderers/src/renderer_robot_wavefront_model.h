#ifndef HUSKY_MODEL_RENDERER_H_
#define HUSKY_MODEL_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_husky_model_renderer(BotViewer *viewer, int render_priority,
                                BotParam * param, BotFrames * frames);

#ifdef __cplusplus
}
#endif

#endif
