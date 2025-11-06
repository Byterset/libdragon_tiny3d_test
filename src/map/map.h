#ifndef __SCENE_MAP_H__
#define __SCENE_MAP_H__

#include "../math/transform.h"
#include "../math/vector2.h"
#include "../render/render_batch.h"
#include "../render/renderable.h"
#include "../render/model.h"
#include "../resource/model_cache.h"


struct map {
    Transform transform;
    struct renderable renderable;
    struct model* model;
};

void map_init(struct map* map);

void map_render(struct map* map, struct render_batch* batch);

void map_destroy(struct map* map);

#endif