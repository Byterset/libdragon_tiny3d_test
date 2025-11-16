#ifndef __OBJECT_BALL_H__
#define __OBJECT_BALL_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../math/mathf.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/physics_object.h"
#include "../../scene/scene_definition.h"

struct ball {
    Transform transform;
    struct renderable renderable;
    physics_object physics;
};

void ball_init(struct ball* ball, struct generic_object_pos_definition* def);

void ball_destroy(struct ball* ball);

#endif