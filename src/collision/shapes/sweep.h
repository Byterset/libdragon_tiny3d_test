#ifndef __COLLISION_SHAPE_SWEEP_H__
#define __COLLISION_SHAPE_SWEEP_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

void sweep_support_function(const void* data, const Vector3* direction, Vector3* output);
void sweep_bounding_box(const void* data, const Quaternion* rotation, AABB* box);


#define SWEEP_COLLIDER(range_x, range_y, r, hh)          \
    .gjk_support_function = sweep_support_function, \
    .bounding_box_calculator = sweep_bounding_box,  \
    .inertia_calculator = NULL,     \
    .shape_type = COLLISION_SHAPE_SWEEP, \
    .shape_data = {                          \
        .sweep = {                       \
            .range = {{range_x, range_y}},    \
            .radius = r,    \
            .half_height = hh,    \
        },                             \
    }

#endif