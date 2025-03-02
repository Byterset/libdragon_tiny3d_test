#ifndef __OBJECT_SODA_CAN_H__
#define __OBJECT_SODA_CAN_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../render/renderable.h"
#include "../../scene/scene_definition.h"

struct soda_can {
    Transform transform;
    struct renderable renderable;
};

void soda_can_init(struct soda_can* soda_can, struct generic_object_pos_definition* def);

void soda_can_destroy(struct soda_can* soda_can);

#endif