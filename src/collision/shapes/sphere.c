#include "sphere.h"

#include "../physics_object.h"
#include <math.h>

#define SQRT_1_3  0.577350269f

void sphere_support_function(void* data, struct Vector3* direction, struct Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = (union physics_object_collision_shape_data*)&object->collision->shape_data;

    float radius = shape_data->sphere.radius;

    vector3Normalize(direction, output);
    vector3Scale(output, output, radius);
}

void sphere_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    vector3Scale(&gOneVec, &box->max, shape_data->sphere.radius);
    vector3Scale(&gOneVec, &box->min, -shape_data->sphere.radius);
}