#include "sphere.h"

#include "../physics_object.h"
#include <math.h>

#define SQRT_1_3  0.577350269f

void sphere_support_function(void* data, Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = (union physics_object_collision_shape_data*)&object->collision->shape_data;

    float radius = shape_data->sphere.radius;

    float distance = fabsf(direction->x);
    output->x = direction->x > 0.0f ? radius : -radius;
    output->y = 0.0f;
    output->z = 0.0f;

    for (int i = 1; i < 3; ++i) {
        float distanceCheck = fabsf(direction->v[i]);

        if (distanceCheck > distance) {
            distance = distanceCheck;
            *output = gZeroVec;
            if (direction->v[i] > 0.0f) {
                output->v[i] = radius;
            } else {
                output->v[i] = -radius;
            }
        }
    }
}

void sphere_bounding_box(void* data, Quaternion* rotation, AABB* box) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    vector3Scale(&gOneVec, &box->max, shape_data->sphere.radius);
    vector3Scale(&gOneVec, &box->min, -shape_data->sphere.radius);
}

void sphere_inertia_tensor(void* data, float mass, Vector3* out) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;
    float radius = shape_data->sphere.radius;

    // Inertia tensor for a solid sphere: I = (2/5) * mass * r²
    // Same for all axes due to rotational symmetry
    float inertia = 0.4f * mass * radius * radius;

    out->x = inertia;
    out->y = inertia;
    out->z = inertia;
}