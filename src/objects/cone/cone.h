#ifndef __OBJECT_CONE_H__
#define __OBJECT_CONE_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../math/mathf.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/physics_object.h"
#include "../../scene/scene_definition.h"

struct cone {
    Transform transform;
    struct renderable renderable;
    struct physics_object physics;
};

void cone_init(struct cone* box, struct generic_object_pos_definition* def);

void cone_destroy(struct cone* box);

#endif