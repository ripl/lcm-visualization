#include <glib.h>

#include <bot_core/glib_util.h>
#include <bot_core/math_util.h>

#include <lcmtypes/ripl_gridmap_tile_t.h>
#include <gridmap_utils/gridmap.h>


#include "tile_set.h"

struct tile_texture *tile_texture_create(const ripl_gridmap_tile_t *tile)
{
    struct tile_texture *tt = (struct tile_texture*) calloc(1, sizeof(struct tile_texture));

    tt->tile = ripl_gridmap_tile_t_copy(tile);
    tt->texture_valid = 0;
    return tt;
}

void tile_texture_destroy(struct tile_texture *tt)
{
    if (tt->texture_valid)
        glDeleteTextures(1, &tt->texture_id);

    ripl_gridmap_tile_t_destroy(tt->tile);
    free(tt);
}

//static void
//tile_destroy_helper(gpointer data, gpointer user_data)
//{
//    struct tile_texture *tt0 = (struct tile_texture*) data;
//    struct tile_set *ts = (struct tile_set*) user_data;
//
//    if (tt0->tile->generation != ts->receiving_generation &&
//        tt0->tile->generation != ts->drawing_generation)
//    {
//        tile_texture_destroy(tt0);
//        //g_ptr_array_remove_index_fast(ts->tiles, i);
//        g_ptr_array_remove_fast(ts->tiles, tt0);
//    }
//
//    return;
//}



void tile_set_process_new_tile(struct tile_set *ts, const ripl_gridmap_tile_t *tile)
{
    struct tile_texture *tt = tile_texture_create(tile);

    if (ts->receiving_generation != tt->tile->generation)
    {
        ts->drawing_generation = ts->receiving_generation;
        ts->receiving_generation = tt->tile->generation;
    }
//    g_ptr_array_foreach(ts->tiles, tile_destroy_helper, ts);

    for (int i = 0; i < ts->tiles->len;)
    {
        struct tile_texture *tt0 = g_ptr_array_index(ts->tiles, i);

        if (tt0->tile->generation != ts->receiving_generation &&
            tt0->tile->generation != ts->drawing_generation)
        {
            g_ptr_array_remove_index_fast(ts->tiles, i);
            tile_texture_destroy(tt0);
        }
        else
        {
            i++;
        }
    }

    g_ptr_array_add(ts->tiles, tt);
}

// compute an RGBA bitmap for a 8bit paletted image. You must
// preallocate out as 4*width*height
static void convert_map_color(int width, int height, const uint8_t *in,
                              const uint32_t *bgra,
                              uint32_t *out)
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int offset = y*width + x;

            int idx = in[offset];

            if (bgra) {
                // indexed color
                out[offset] = bgra[idx];
            } else {
                // greyscale
                out[offset] = (idx<<0)|(idx<<8)|(idx<<16)|(255<<24);
            }
        }
    }
}

void tile_set_draw_tile(struct tile_set *ts, struct tile_texture *tt)
{
    if (!tt->texture_valid) {
        glGenTextures( 1, &tt->texture_id );
        tt->texture_valid = 1;

        uint8_t *data8 = (uint8_t*) malloc(tt->tile->width * tt->tile->height);
        uint32_t *data32 = (uint32_t*) malloc(4 * tt->tile->width * tt->tile->height);

        gridmap_decode_base(data8, tt->tile->width, tt->tile->height, tt->tile->data, tt->tile->datalen);

        convert_map_color(tt->tile->width, tt->tile->height, data8,
                          ts->bgra,
                          data32);

        glBindTexture (GL_TEXTURE_2D, tt->texture_id);
        glTexImage2D (GL_TEXTURE_2D,
                      0,
                      GL_RGBA8,
                      tt->tile->width, tt->tile->height,
                      0,
                      GL_BGRA, // GL_COLOR_INDEX,
                      GL_UNSIGNED_BYTE, // GL_UNSIGNED_BYTE,
                      data32);
        glBindTexture (GL_TEXTURE_2D, 0);

        free(data8);
        free(data32);
    }

    glPushAttrib (GL_ENABLE_BIT);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable (GL_DEPTH_TEST);
    glEnable (GL_TEXTURE_2D);

    glBindTexture (GL_TEXTURE_2D, tt->texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                    GL_NEAREST);

    double x0 = tt->tile->x0;
    double x1 = tt->tile->x0 + tt->tile->width * tt->tile->meters_per_pixel;
    double y0 = tt->tile->y0;
    double y1 = tt->tile->y0 + tt->tile->height * tt->tile->meters_per_pixel;

    int d = 1;
    glBegin (GL_QUADS);
    glTexCoord2i (0,0);
    glVertex2d (x0, y0);
    glTexCoord2i (0, d);
    glVertex2d (x0, y1);
    glTexCoord2i (d, d);
    glVertex2d (x1, y1);
    glTexCoord2i (d, 0);
    glVertex2d (x1, y0);
    glEnd ();

    glBindTexture (GL_TEXTURE_2D, 0);
    glPopAttrib ();
    glEnable (GL_DEPTH_TEST);
}

struct tile_set *tile_set_create()
{
    return tile_set_create_colormap(NULL, NULL, NULL, NULL, 0);
}

struct tile_set *tile_set_create_colormap(const float *r, const float *g,
                                          const float *b, const float *a, int ncolors)
{
    struct tile_set *ts = (struct tile_set*) calloc(1, sizeof(struct tile_set));
    ts->tiles = g_ptr_array_new();
    ts->drawing_generation = -100;
    ts->ncolors = ncolors;
    ts->bgra = calloc(ncolors, sizeof(uint32_t));

    for (int i = 0; i < ncolors; i++) {
        int rv = r[i]*255, bv = b[i]*255, gv = g[i]*255, av = a[i]*255;
        ts->bgra[i] = (bv<<0) | (gv<<8) | (rv<<16) | (av<<24);
    }

    return ts;
}

void tile_set_destroy(struct tile_set *ts)
{
    for (int i = 0; i < ts->tiles->len; i++) {
        struct tile_texture *tt = g_ptr_array_index(ts->tiles, i);
        tile_texture_destroy(tt);
    }
    g_ptr_array_free(ts->tiles, TRUE);
    free(ts->bgra);
    free(ts);
}

void
tile_set_draw_tile_helper (gpointer data, gpointer user_data)
{
    struct tile_texture *tt = (struct tile_texture*) data;
    struct tile_set *ts = (struct tile_set*) user_data;

    if (tt->tile->generation == ts->drawing_generation) {
        tile_set_draw_tile(ts, tt);
    }

}

void tile_set_draw(struct tile_set *ts)
{
    if (!ts) {
        fprintf (stdout, "tile_set_draw: ts is NULL\n");
        return;
    }
    g_ptr_array_foreach(ts->tiles, tile_set_draw_tile_helper, ts);

    //for (int tidx = 0; tidx < g_ptr_array_size(ts->tiles); tidx++) {
    //    struct tile_texture *tt = g_ptr_array_index(ts->tiles, tidx);

    //    if (tt->tile->generation == ts->drawing_generation)
    //        tile_set_draw_tile(ts, tt);
    //}
}
