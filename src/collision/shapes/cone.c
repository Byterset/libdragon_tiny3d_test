#include "cone.h"

#include "../physics_object.h"
#include "../../math/minmax.h"
#include "../../math/mathf.h"
#include <math.h>

void cone_support_function(void* data, Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = (union physics_object_collision_shape_data*)&object->collision->shape_data;

    output->x = direction->x > 0.0f ? shape_data->cone.radius : -shape_data->cone.radius;
    output->z = direction->z > 0.0f ? shape_data->cone.radius : -shape_data->cone.radius;
    output->y = -shape_data->cone.half_height;

    if (vector3Dot(output, direction) < 0) {
        *output = gZeroVec;
        output->y = shape_data->cone.half_height;
    }
}

void cone_bounding_box(void* data, Quaternion* rotation, AABB* box) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;
    
    // Get capsule dimensions
    float half_height = shape_data->cone.half_height;
    float radius = shape_data->cone.radius;

    // Define the capsule's central axis in local space
    Vector3 corners[5] = { 
        {{-radius, -half_height, -radius }},
        {{ radius, -half_height, -radius }},
        {{-radius, -half_height,  radius }},
        {{ radius, -half_height,  radius }},
        {{ 0.0f,  half_height, 0.0f }}
    };
    Vector3 new_min;
    Vector3 new_max;

    // Initialize new_min and new_max with the first rotated corner
    Vector3 rotated_corner;
    if(object->rotation)
        quatMultVector(object->rotation, &corners[0], &rotated_corner);
    else
        vector3Copy(&corners[0], &rotated_corner);
    new_min = rotated_corner;
    new_max = rotated_corner;

    for(int i = 1; i < 5; i++){
        if(object->rotation)
            quatMultVector(object->rotation, &corners[i], &rotated_corner);
        else
            vector3Copy(&corners[i], &rotated_corner);

        // Update new_min and new_max
        vector3Min(&new_min, &rotated_corner, &new_min);
        vector3Max(&new_max, &rotated_corner, &new_max);
    }

    box->min = new_min;
    box->max = new_max;
}