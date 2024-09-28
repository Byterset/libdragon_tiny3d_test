#include "skybox3d.h"

#include <libdragon.h>

#include "../math/vector2.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../render/defs.h"
#include "../resource/sprite_cache.h"

void skybox_update(void* data){

}

void skybox_custom_render(void* data, struct render_batch* batch) {
    struct skybox* skybox = (struct skybox*)data;
    render_batch_add_equidistant(batch, skybox->block);
}

//TODO: Add the rest of the skybox faces and calculate the subdivisions on init
void skybox_init(struct skybox* skybox) {
    // Allocate vertices (make sure to have an uncached pointer before passing it to the API!)
    // For performance reasons, 'T3DVertPacked' contains two vertices at once in one struct.
    skybox->verts = malloc_uncached(sizeof(T3DVertPacked) * 8);

    skybox->texture_front = sprite_cache_load("rom:/images/skybox/skybox_front.sprite");
    int16_t max_UV_x = (skybox->texture_front->width << 5) - 1;
    int16_t max_UV_y = (skybox->texture_front->height << 5) - 1;


    skybox->verts[0] = (T3DVertPacked){
        .posA = {-3, 3, 3}, // 0
        .rgbaA = 0xFFFFFFFF,
        .stA = {0, 0},
        .posB = {-1, 3, 3}, // 1
        .rgbaB = 0xFFFFFFFF,
        .stB = {max_UV_x, 0},
    };
    skybox->verts[1] = (T3DVertPacked){
        .posA = {1, 3, 3}, // 2
        .stA = {0, 0},
        .rgbaA = 0xFFFFFFFF,
        .posB = {3, 3, 3}, // 3
        .rgbaB = 0xFFFFFFFF,
        .stB = {max_UV_x, 0},
    };
    skybox->verts[2] = (T3DVertPacked){
        .posA = {-3, 1, 3}, // 4
        .rgbaA = 0xFFFFFFFF,
        .stA = {0, max_UV_y},
        .posB = {-1, 1, 3}, // 5
        .rgbaB = 0xFFFFFFFF,
        .stB = {max_UV_x, max_UV_y},
    };
    skybox->verts[3] = (T3DVertPacked){
        .posA = {1, 1, 3}, // 6
        .stA = {0, max_UV_y},
        .rgbaA = 0xFFFFFFFF,
        .posB = {3, 1, 3}, // 7
        .rgbaB = 0xFFFFFFFF,
        .stB = {max_UV_x, max_UV_y},
    };
    skybox->verts[4] = (T3DVertPacked){
        .posA = {-3, -1, 3}, // 8
        .rgbaA = 0xFFFFFFFF,

        .posB = {-1, -1, 3}, // 9
        .rgbaB = 0xFFFFFFFF,

    };
    skybox->verts[5] = (T3DVertPacked){
        .posA = {1, -1, 3}, // 10
        .rgbaA = 0xFFFFFFFF,

        .posB = {3, -1, 3}, // 11
        .rgbaB = 0xFFFFFFFF,

    };
    skybox->verts[6] = (T3DVertPacked){
        .posA = {-3, -3, 3}, // 12
        .rgbaA = 0xFFFFFFFF,

        .posB = {-1, -3, 3}, // 13
        .rgbaB = 0xFFFFFFFF,

    };
    skybox->verts[7] = (T3DVertPacked){
        .posA = {1, -3, 3}, // 14
        .rgbaA = 0xFFFFFFFF,
        .posB = {3, -3, 3}, // 15
        .rgbaB = 0xFFFFFFFF,
    };


    rspq_block_begin();
    rdpq_sprite_upload(TILE0, skybox->texture_front, NULL);
    t3d_vert_load(skybox->verts, 0, 16); 
    //first row of tiles
    t3d_tri_draw(0, 1, 4);
    t3d_tri_draw(1, 4, 5);

    t3d_tri_draw(1, 2, 5);
    t3d_tri_draw(2, 5, 6);

    t3d_tri_draw(2, 3, 6);
    t3d_tri_draw(3, 6, 7);

    //second row of tiles
    t3d_tri_draw(4, 5, 8);
    t3d_tri_draw(5, 8, 9);

    t3d_tri_draw(5, 6, 9);
    t3d_tri_draw(6, 9, 10);

    t3d_tri_draw(6, 7, 10);
    t3d_tri_draw(7, 10, 11);


    //third row of tiles
    t3d_tri_draw(8, 9, 12);
    t3d_tri_draw(9, 12, 13);

    t3d_tri_draw(9, 10, 13);
    t3d_tri_draw(10, 13, 14);

    t3d_tri_draw(10, 11, 14);
    t3d_tri_draw(11, 14, 15);

    t3d_tri_sync(); // after each batch of triangles, a sync is needed
    skybox->block = rspq_block_end();

    render_scene_add(&gZeroVec, 80.0f, skybox_custom_render, skybox);
}



void skybox_destroy(struct skybox* skybox) {
    free(skybox->verts);
    rspq_block_free(skybox->block);
}