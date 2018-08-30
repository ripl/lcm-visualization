#ifndef ROBOT_STATUS_RENDERER_H_
#define ROBOT_STATUS_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_robot_status (BotViewer *viewer, BotParam *param, int priority);

#ifdef __cplusplus
}
#endif

#endif
