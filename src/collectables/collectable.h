#ifndef __COLLECTABLES_COLLECTABLE_H__
#define __COLLECTABLES_COLLECTABLE_H__

#include "../math/vector3.h"
#include "../collision/physics_object.h"
#include "../scene/scene_definition.h"
#include "../math/transform_single_axis.h"
#include "../render/renderable.h"

struct collectable {
    struct TransformSingleAxis transform;
    struct renderable_single_axis renderable;
    struct physics_object physics;
    enum collectable_type collectable_type;
    uint16_t collectable_sub_type;
};

void collectable_assets_load();

void collectable_init(struct collectable* collectable, struct collectable_definition* definition);
void collectable_collected(struct collectable* collectable);
void collectable_destroy(struct collectable* collectable);

struct collectable* collectable_get(entity_id id);

#endif