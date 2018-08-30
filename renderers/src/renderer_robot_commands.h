#ifndef ROBOT_COMMANDS_RENDERER_H_
#define ROBOT_COMMANDS_RENDERER_H_

#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_robot_commands (BotViewer *viewer, int priority);


#ifdef __cplusplus
}
#endif

#endif
