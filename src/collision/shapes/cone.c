#include "cylinder.h"

#include "../dynamic_object.h"
#include "../../math/minmax.h"
#include <math.h>

void cone_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct dynamic_object *object = (struct dynamic_object *)data;
    union dynamic_object_type_data *shape_data = (union dynamic_object_type_data *)&object->type->data;

    float projected_length = fabsf(direction->y);
    struct Vector3 support_point_local;
    if(direction->x * direction->x + direction->z * direction->z < 0.0001f){
        output->x = 0.0f;
        output->y = shape_data->cone.height;
        output->z = 0.0f;
    } else if(projected_length <= shape_data->cone.height){ 
        float scale_factor = shape_data->cone.radius / sqrtf(direction->x * direction->x + direction->z * direction->z);
        output->x = direction->x * scale_factor;
        output->y = shape_data->cone.height;
        output->z = direction->z * scale_factor;
    }
    else {
        output->x = direction->x * shape_data->cone.radius / projected_length;
        output->y = -shape_data->cone.height;
        output->z = direction->z * shape_data->cone.radius / projected_length;
    }
}

void cone_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = &object->type->data;
    
    // Get cone dimensions
    float half_height = shape_data->cone.height / 2.0f;
    float radius = shape_data->cone.radius;

    // Define the cone's central axis in local space
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