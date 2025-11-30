#ifndef __OBJECT_PYRAMID_H__
#define __OBJECT_PYRAMID_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../math/mathf.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/physics_object.h"
#include "../../scene/scene_definition.h"

struct pyramid {
    Transform transform;
    struct renderable renderable;
    physics_object physics;
};

void pyramid_init(struct pyramid* box, struct generic_object_pos_definition* def);

void pyramid_destroy(struct pyramid* box);

#endif