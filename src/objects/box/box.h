#ifndef __SCENE_BOX_H__
#define __SCENE_BOX_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/dynamic_object.h"
#include "../../scene/scene_definition.h"

struct box {
    struct Transform transform;
    struct renderable renderable;
    struct dynamic_object collision;
    struct Vector2 look_direction;
};

void box_init(struct box* box, struct box_definition* def);

void box_destroy(struct box* box);

#endif