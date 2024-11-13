#ifndef __OBJECT_BOX_H__
#define __OBJECT_BOX_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../math/mathf.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/physics_object.h"
#include "../../scene/scene_definition.h"

struct box {
    Transform transform;
    struct renderable renderable;
    struct physics_object physics;
};

void box_init(struct box* box, struct box_definition* def);

void box_destroy(struct box* box);

#endif