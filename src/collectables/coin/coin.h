#ifndef __COLLECTABLES_COIN_H__
#define __COLLECTABLES_COIN_H__


#include "../../math/transform.h"
#include "../../math/mathf.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/physics_object.h"
#include "../../scene/scene_definition.h"

struct coin {
    struct Transform transform;
    struct renderable renderable;
    struct physics_object physics;
};

void coin_init(struct coin* coin, struct coin_definition* def);

void coin_destroy(struct coin* coin);


#endif // __COLLECTABLES_COIN_H__