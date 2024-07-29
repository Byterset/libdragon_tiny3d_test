#ifndef __SCENE_MAP_H__
#define __SCENE_MAP_H__

#include "../math/transform.h"
#include "../math/vector2.h"
#include "../render/render_batch.h"
#include "../render/renderable.h"


struct map {
    struct Transform transform;
    struct renderable renderable;
};

void map_init(struct map* map);

void map_render(struct map* map, struct render_batch* batch);

void map_destroy(struct map* map);

#endif