#include "cylinder.h"

#include "../dynamic_object.h"
#include <math.h>
#include "sphere.h"

void capsule_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = (union dynamic_object_type_data*)&object->type->data;


    // Step 2: Determine the point along the capsule's central axis (local y-axis) that is furthest in `dir_local` direction
    struct Vector3 axis_point;
    float sign = (direction->y > 0.0f) ? 1.0f : -1.0f;
    axis_point.x = 0.0f;
    axis_point.z = 0.0f;
    axis_point.y = sign * shape_data->capsule.inner_half_height;

    // Step 3: Offset by the capsule's radius in the direction of `dir_local`
    struct Vector3 radius_offset;
    vector3Copy(direction, &radius_offset);
    radius_offset.x *= shape_data->capsule.radius;
    radius_offset.y *= shape_data->capsule.radius;
    radius_offset.z *= shape_data->capsule.radius;

    // Combine the endpoint on the capsule's axis with the radius offset
    output->x = axis_point.x + radius_offset.x;
    output->y = axis_point.y + radius_offset.y;
    output->z = axis_point.z + radius_offset.z;
}


void capsule_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
    union dynamic_object_type_data* shape_data = (union dynamic_object_type_data*)data;
    
    // Get capsule dimensions
    float half_height = shape_data->capsule.inner_half_height;
    float radius = shape_data->capsule.radius;

    // Define the capsule's central axis in local space
    struct Vector3 local_axis = { 0.0f, half_height, 0.0f };

    // Rotate the central axis by the given rotation to get its orientation in world space
    struct Vector3 world_axis;
    if(rotation)
        quatMultVector(rotation, &local_axis, &world_axis);
    else
        vector3Copy(&local_axis, &world_axis);

    // Calculate the extents by projecting the rotated axis and adding the radius
    float extent_x = fabsf(world_axis.x) + radius;
    float extent_y = fabsf(world_axis.y) + radius;
    float extent_z = fabsf(world_axis.z) + radius;

    // Set the AABB min and max based on the calculated extents
    box->min.x = -extent_x;
    box->min.y = -extent_y;
    box->min.z = -extent_z;

    box->max.x = extent_x;
    box->max.y = extent_y;
    box->max.z = extent_z;
}