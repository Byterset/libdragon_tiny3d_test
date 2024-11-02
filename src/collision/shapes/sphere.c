#include "sphere.h"

#include "../dynamic_object.h"
#include <math.h>

#define SQRT_1_3  0.577350269f

void sphere_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = (union dynamic_object_type_data*)&object->type->data;

    float radius = shape_data->sphere.radius;

    vector3Normalize(direction, output);
    vector3Scale(output, output, radius);
}

void sphere_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = &object->type->data;

    vector3Scale(&gOneVec, &box->max, shape_data->sphere.radius);
    vector3Scale(&gOneVec, &box->min, -shape_data->sphere.radius);
}