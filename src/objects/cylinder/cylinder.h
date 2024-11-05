#ifndef __OBJECT_CYLINDER_H__
#define __OBJECT_CYLINDER_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../math/mathf.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/physics_object.h"
#include "../../scene/scene_definition.h"

struct cylinder {
    struct Transform transform;
    struct renderable renderable;
    struct physics_object collision;
};

void cylinder_init(struct cylinder* box, struct cylinder_definition* def);

void cylinder_destroy(struct cylinder* box);

#endif