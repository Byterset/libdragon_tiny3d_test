#include "sweep.h"

#include "../../math/minmax.h"
#include "../physics_object.h"

void sweep_support_function(const void* data, const Vector3* direction, Vector3* output) {
    union physics_object_collision_shape_data* shape_data = (union physics_object_collision_shape_data*)data;

    Vector2 dir_2d;
    dir_2d.x = direction->x;
    dir_2d.y = direction->z;

    Vector2 result_2d = gZeroVec2;
    float distance = 0.0f;

    Vector2 arm_check = shape_data->sweep.range;

    if (direction->x < 0.0f) {
        arm_check.x = -arm_check.x;
    }

    float test = vector2Dot(&dir_2d, &arm_check);

    if (test > distance) {
        distance = test;
        result_2d = arm_check;
    }

    if (dir_2d.y > distance) {
        result_2d = (Vector2){{0.0f, 1.0f}};
    }

    output->x = result_2d.x * shape_data->sweep.radius;
    output->y = direction->y > 0.0f ? shape_data->sweep.half_height : -shape_data->sweep.half_height;
    output->z = result_2d.y * shape_data->sweep.radius;
}

void sweep_bounding_box(const void* data, const Quaternion* rotation, AABB* box) {
    physics_object* object = (physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    // box->min = (Vector3){0.0f, -shape_data->sweep.half_height, 0.0f};
    // box->max = (Vector3){0.0f, shape_data->sweep.half_height, 1.0f};

    // for (int i = 0; i < 2; i += 1) {
    //     Vector2 extent;
    //     extent.x = i == 0 ? shape_data->sweep.range.x : -shape_data->sweep.range.x;
    //     extent.y = shape_data->sweep.range.y;
    //     vector2ComplexMul(rotation, &extent, &extent);
    //     box->min.x = MIN(box->min.x, extent.x);
    //     box->min.z = MIN(box->min.z, extent.y);

    //     box->max.x = MAX(box->max.x, extent.x);
    //     box->max.z = MAX(box->max.z, extent.y);
    // }

    // box->min.x *= shape_data->sweep.radius;
    // box->min.z *= shape_data->sweep.radius;
    // box->max.x *= shape_data->sweep.radius;
    // box->max.z *= shape_data->sweep.radius;
}