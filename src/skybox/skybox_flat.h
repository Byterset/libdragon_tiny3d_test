#ifndef __SCENE_SKYBOX_FLAT_H__
#define __SCENE_SKYBOX_FLAT_H__

#include "../math/transform.h"
#include "../math/vector2.h"
#include "../render/render_batch.h"
#include "../render/renderable.h"
#include "../render/model.h"
#include "../resource/model_cache.h"



struct skybox_flat {
    sprite_t* texture;
    surface_t surface;
};

void skybox_flat_init(struct skybox_flat* skybox);

void skybox_flat_destroy(struct skybox_flat* skybox);


#endif