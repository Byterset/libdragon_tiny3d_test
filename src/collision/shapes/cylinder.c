#include "cylinder.h"

#include "../physics_object.h"
#include "../../math/matrix.h"
#include <math.h>
#include <libdragon.h>

void cylinder_support_function(void* data, Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    Vector3 directionXZ = {{direction->x, 0, direction->z}};

    // Check if the direction in the XZ plane is significant
    float lenSquaredXZ = directionXZ.x * directionXZ.x + directionXZ.z * directionXZ.z;

    if (lenSquaredXZ > 0.0001f) {
        // Normalize the XZ direction
        float invLen = 1.0f / sqrtf(lenSquaredXZ);
        directionXZ.x *= invLen;
        directionXZ.z *= invLen;
        
        // Get the furthest point on the circle in the XZ plane
        output->x = shape_data->cylinder.radius * directionXZ.x;
        output->z = shape_data->cylinder.radius * directionXZ.z;
    } else {
        // Direction is nearly vertical, point is on cylinder axis
        output->x = 0;
        output->z = 0;
    }

    // Determine which end cap is furthest in the direction
    if (direction->y > 0)
    {
        output->y = shape_data->cylinder.half_height; // Top cap
    }
    else
    {
        output->y = -shape_data->cylinder.half_height; // Bottom cap
    }
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