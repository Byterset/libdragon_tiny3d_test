#include "cylinder.h"

#include "../dynamic_object.h"
#include <math.h>

#define SQRT_1_2   0.707106781f

void cylinder_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = (union dynamic_object_type_data*)&object->type->data;


    // Step 2: Determine the point along the cylinders's central axis (local y-axis) that is furthest in `direction` direction
    struct Vector3 axis_point;
    float sign = (direction->y > 0.0f) ? 1.0f : -1.0f;
    axis_point.x = 0.0f;
    axis_point.z = 0.0f;
    axis_point.y = sign * shape_data->cylinder.half_height;

    // Step 3: Offset by the cylinders's radius in the direction of `direction`
    struct Vector3 radius_offset;
    vector3Copy(direction, &radius_offset);
    radius_offset.x *= shape_data->cylinder.radius;
    radius_offset.z *= shape_data->cylinder.radius;

    // Combine the endpoint on the cylinders's axis with the radius offset
    output->x = axis_point.x + radius_offset.x;
    output->y = axis_point.y;
    output->z = axis_point.z + radius_offset.z;

}

void cylinder_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = &object->type->data;
    
    // Get cylinder dimensions
    float half_height = shape_data->cylinder.half_height;
    float radius = shape_data->cylinder.radius;

    // Define the cylinders's central axis in local space
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