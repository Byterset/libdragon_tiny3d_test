#ifndef __SCENE_SKYBOX3D_H__
#define __SCENE_SKYBOX3D_H__

#include "../math/transform.h"
#include "../math/vector2.h"
#include "../render/render_batch.h"
#include "../render/renderable.h"
#include "../render/model.h"
#include "../resource/model_cache.h"

#define SKYBOX_FACE_SUBDIVISIONS 3
#define SKYBOX_GRID_SIZE SKYBOX_FACE_SUBDIVISIONS + 1
#define SKYBOX_TOTAL_VERT_COUNT (SKYBOX_GRID_SIZE * SKYBOX_GRID_SIZE * SKYBOX_GRID_SIZE)


struct skybox_face {

};

struct skybox {
    T3DVertPacked *verts;
    sprite_t* texture_front;
    rspq_block_t *block;
};

void skybox_init(struct skybox* skybox);

void skybox_destroy(struct skybox* skybox);


#endif