#ifndef _TILE_SET_H
#define _TILE_SET_H

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <lcmtypes/gmlcm_gridmap_tile_t.h>

struct tile_set
{
    const char *channel;
    GPtrArray  *tiles;
    int64_t     receiving_generation;
    int64_t     drawing_generation;

    // for color mapping
    int        ncolors;
    uint32_t   *bgra;  // allocated by tile_set_create_colormap
    
};

struct tile_texture 
{ 
    gmlcm_gridmap_tile_t *tile;
    GLuint                texture_id;
    int                   texture_valid;
};

struct tile_set *tile_set_create();
struct tile_set *tile_set_create_colormap(const float *r, const float *g,
                                          const float *b, const float *a, int ncolors);
void tile_set_destroy(struct tile_set *ts);
void tile_set_process_new_tile(struct tile_set *ts, const gmlcm_gridmap_tile_t *tile);
void tile_set_draw(struct tile_set *ts);

#endif

