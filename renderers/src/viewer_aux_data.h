#ifndef __viewer_aux_data_h__
#define __viewer_aux_data_h__

#include <stdint.h>
#include <lcm/lcm.h>
#include <bot_vis/viewer.h>
#include <bot_core/camtrans.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _ViewerAuxData ViewerAuxData;

struct _ViewerAuxData
{
    BotViewer *viewer;
    lcm_t *lcm;
    int logplayer_udp_port;
    int simulation_flag;
    GPtrArray *camtranses;
    BotCamTrans *active_camtrans;
    char **cam_names;
    BotFrames *frames;
};

ViewerAuxData * get_viewer_aux_data (BotViewer *viewer);

#ifdef __cplusplus
}
#endif

#endif
