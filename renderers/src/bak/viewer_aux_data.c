#include <glib.h>

#include "viewer_aux_data.h"

ViewerAuxData * 
get_viewer_aux_data (BotViewer *viewer)
{
    return (ViewerAuxData*) g_object_get_data (G_OBJECT(viewer), 
            "viewer:aux-data");
}
