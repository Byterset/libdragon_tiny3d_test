#include "cylinder.h"

#include "../physics_object.h"
#include "../../math/matrix.h"
#include <math.h>
#include <libdragon.h>

#define SQRT_1_2 0.707106781f

void cylinder_support_function(void* data, Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    float abs_x = fabsf(direction->x);
    float abs_z = fabsf(direction->z);
    float angle_dot = (abs_x + abs_z) * SQRT_1_2;

    if (abs_x < angle_dot && abs_z < angle_dot) {
        output->x = direction->x > 0.0f ? SQRT_1_2 * shape_data->cylinder.radius : -SQRT_1_2 * shape_data->cylinder.radius;
        output->z = direction->z > 0.0f ? SQRT_1_2 * shape_data->cylinder.radius : -SQRT_1_2 * shape_data->cylinder.radius;
    } else if (abs_x > abs_z) {
        output->x = direction->x > 0.0f ? shape_data->cylinder.radius : -shape_data->cylinder.radius;
        output->z = 0.0f;
    } else {
        output->x = 0.0f;
        output->z = direction->z > 0.0f ? shape_data->cylinder.radius : -shape_data->cylinder.radius;
    }

    output->y = direction->y > 0.0f ? shape_data->cylinder.half_height : -shape_data->cylinder.half_height;

    // // Project direction onto XZ plane to find the furthest point on the circular cross-section
    // // Use the direction components directly scaled by radius, similar to capsule
    // float lenSquaredXZ = direction->x * direction->x + direction->z * direction->z;

    // if (lenSquaredXZ > 1e-8f) {
    //     // Normalize the XZ direction and scale by radius
    //     float invLen = 1.0f / sqrtf(lenSquaredXZ);
    //     output->x = shape_data->cylinder.radius * direction->x * invLen;
    //     output->z = shape_data->cylinder.radius * direction->z * invLen;
    // } else {
    //     // Direction is purely vertical - any point on the circle is equally valid
    //     // Return center of cap (0, 0) in XZ plane
    //     output->x = 0.0f;
    //     output->z = 0.0f;
    // }

    // // Determine which end cap is furthest in the direction
    // if (direction->y > 0)
    // {
    //     output->y = shape_data->cylinder.half_height; // Top cap
    // }
    // else
    // {
    //     output->y = -shape_data->cylinder.half_height; // Bottom cap
    // }
}

void cylinder_bounding_box(void* data, Quaternion* rotation, AABB* box) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    // Get capsule dimensions
    float half_height = shape_data->cylinder.half_height;
    float radius = shape_data->cylinder.radius;

    // Define the cylinders local aabb
    box->min = (Vector3){{ -radius, -half_height, -radius }};
    box->max = (Vector3){{  radius,  half_height,  radius }};


    // Rotate the local aabb if needed with Arvo's method
    if (rotation) {
        Matrix4x4 rotMat;
        AABB aabb_rotated = {0};
        quatToMatrix(rotation, rotMat.m);
        
        //for all three axes
        for(int i = 0; i < 3; i++){
            // Form extent by summing smaller and larger terms respectively
            for (int j = 0; j < 3; j++)
            {
                float a = rotMat.m[i][j] * box->min.v[j];
                float b = rotMat.m[i][j] * box->max.v[j];
                
                aabb_rotated.min.v[i] += (a < b) ? a : b;
                aabb_rotated.max.v[i] += (a < b) ? b : a;
            }
            
        }
        box->min = aabb_rotated.min;
        box->max = aabb_rotated.max;
    }

}

void cylinder_inertia_tensor(void* data, float mass, Vector3* out) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    float radius = shape_data->cylinder.radius;
    float half_height = shape_data->cylinder.half_height;
    float height = 2.0f * half_height;

    // Inertia tensor for a solid cylinder oriented along y-axis:
    // Ixx = Izz = (1/12) * mass * (3*r² + h²)  (perpendicular to cylinder axis)
    // Iyy = (1/2) * mass * r²  (along cylinder axis)
    float r_sq = radius * radius;
    float h_sq = height * height;

    float perpendicular_inertia = mass * (3.0f * r_sq + h_sq) / 12.0f;
    float axial_inertia = 0.5f * mass * r_sq;

    out->x = perpendicular_inertia;
    out->y = axial_inertia;
    out->z = perpendicular_inertia;
}