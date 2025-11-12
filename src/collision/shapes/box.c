#include "box.h"

#include "../physics_object.h"
#include <math.h>
#include "../../math/matrix.h"
#include "../../render/defs.h"

void box_support_function(void* data, Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = (union physics_object_collision_shape_data*)&object->collision->shape_data;
    output->x = direction->x > 0.0f ? shape_data->box.half_size.x : -shape_data->box.half_size.x;
    output->y = direction->y > 0.0f ? shape_data->box.half_size.y : -shape_data->box.half_size.y;
    output->z = direction->z > 0.0f ? shape_data->box.half_size.z : -shape_data->box.half_size.z;
}

void box_bounding_box(void* data, Quaternion* rotation, AABB* box) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;
    Vector3* half_size = &shape_data->box.half_size;
    float extent_x, extent_y, extent_z;

    // Rotate the basis vectors according to the given rotation
    if(rotation){

        Matrix4x4 rot_mat;
        quatToMatrix(rotation, rot_mat.m);
        
        Matrix4x4 scale = {0};
        scale.m[0][0] = half_size->x;
        scale.m[1][1] = half_size->y;
        scale.m[2][2] = half_size->z;
        scale.m[3][3] = 1;

        Matrix4x4 world_bounds;
        matrixMul(scale.m, rot_mat.m, world_bounds.m);

        // Calculate the extents in each axis
        extent_x = fabsf(world_bounds.m[0][0]) + fabsf(world_bounds.m[0][1]) + fabsf(world_bounds.m[0][2]);
        extent_y = fabsf(world_bounds.m[1][0]) + fabsf(world_bounds.m[1][1]) + fabsf(world_bounds.m[1][2]);
        extent_z = fabsf(world_bounds.m[2][0]) + fabsf(world_bounds.m[2][1]) + fabsf(world_bounds.m[2][2]);
    }
    else{
        extent_x = fabsf(half_size->x);
        extent_y = fabsf(half_size->y);
        extent_z = fabsf(half_size->z);
    }

    // Set the min and max of the AABB
    box->min.x = -extent_x;
    box->min.y = -extent_y;
    box->min.z = -extent_z;

    box->max.x = extent_x;
    box->max.y = extent_y;
    box->max.z = extent_z;
}

void box_inertia_tensor(void* data, float mass, Vector3* out) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;
    Vector3* half_size = &shape_data->box.half_size;

    // Inertia tensor for a box with half-sizes (hx, hy, hz):
    // Ixx = (1/3) * mass * (hy² + hz²)
    // Iyy = (1/3) * mass * (hx² + hz²)
    // Izz = (1/3) * mass * (hx² + hy²)
    float hx_sq = half_size->x * half_size->x;
    float hy_sq = half_size->y * half_size->y;
    float hz_sq = half_size->z * half_size->z;

    float scale = mass / 3.0f;

    out->x = scale * (hy_sq + hz_sq);
    out->y = scale * (hx_sq + hz_sq);
    out->z = scale * (hx_sq + hy_sq);
}