#include "cone.h"

#include "../physics_object.h"
#include "../../math/minmax.h"
#include "../../math/mathf.h"
#include <math.h>

void cone_support_function(void* data, struct Vector3* direction, struct Vector3* output) {
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

void cone_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
  struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;
    
    // Get cone dimensions
    float half_height = shape_data->cone.half_height;
    float radius = shape_data->cone.radius;

    // Define the cones's central axis in local space
    struct Vector3 local_axis = { 0.0f, half_height, 0.0f };

    // Rotate the central axis by the given rotation to get its orientation in world space
    struct Vector3 world_axis;
    if(rotation)
        quatMultVector(rotation, &local_axis, &world_axis);
    else
        vector3Copy(&local_axis, &world_axis);

    // Calculate the extents by projecting the rotated axis and adding the radius
    float extent_x = fabsf(world_axis.x) + radius;
    float extent_y = fabsf(world_axis.y);
    float extent_z = fabsf(world_axis.z) + radius;

    // Set the AABB min and max based on the calculated extents
    box->min.x = -extent_x;
    box->min.y = -extent_y;
    box->min.z = -extent_z;

    box->max.x = extent_x;
    box->max.y = extent_y;
    box->max.z = extent_z;
}