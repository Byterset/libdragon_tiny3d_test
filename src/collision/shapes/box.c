#include "cylinder.h"

#include "../dynamic_object.h"
#include <math.h>
#include "../../render/defs.h"

void box_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = (union dynamic_object_type_data*)&object->type->data;

    output->x = direction->x > 0.0f ? shape_data->box.half_size.x : -shape_data->box.half_size.x;
    output->y = direction->y > 0.0f ? shape_data->box.half_size.y : -shape_data->box.half_size.y;
    output->z = direction->z > 0.0f ? shape_data->box.half_size.z : -shape_data->box.half_size.z;
}

void box_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box) {
    struct dynamic_object* object = (struct dynamic_object*)data;
    union dynamic_object_type_data* shape_data = &object->type->data;
    struct Vector3* half_size = &shape_data->box.half_size;

    // Define the local basis vectors for the oriented box
    struct Vector3 local_x = { half_size->x, 0.0f, 0.0f };
    struct Vector3 local_y = { 0.0f, half_size->y, 0.0f };
    struct Vector3 local_z = { 0.0f, 0.0f, half_size->z };

    // Rotate the basis vectors according to the given rotation
    struct Vector3 world_x, world_y, world_z;
    if(rotation){
        quatMultVector(rotation, &local_x, &world_x);
        quatMultVector(rotation, &local_y, &world_y);
        quatMultVector(rotation, &local_z, &world_z);
    }
    else{
        vector3Copy(&local_x, &world_x);
        vector3Copy(&local_y, &world_y);
        vector3Copy(&local_z, &world_z);
    }


    // Calculate the extents in each axis
    float extent_x = fabsf(world_x.x) + fabsf(world_y.x) + fabsf(world_z.x);
    float extent_y = fabsf(world_x.y) + fabsf(world_y.y) + fabsf(world_z.y);
    float extent_z = fabsf(world_x.z) + fabsf(world_y.z) + fabsf(world_z.z);

    // Set the min and max of the AABB
    box->min.x = -extent_x;
    box->min.y = -extent_y;
    box->min.z = -extent_z;

    box->max.x = extent_x;
    box->max.y = extent_y;
    box->max.z = extent_z;
}