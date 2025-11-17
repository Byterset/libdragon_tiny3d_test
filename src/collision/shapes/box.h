#ifndef __COLLISION_SHAPE_BOX_H__
#define __COLLISION_SHAPE_BOX_H__

#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../../math/aabb.h"

/// @brief GJK Support function (see gjk_support_function) for the box primitive.
///
/// Will return the point on the box boundary furthest in the input direction.
/// @param data pointer to the physics_object holding the box collider
/// @param direction input direction (is expected to be normalized and rotated if the object has rotation)
/// @param output the pointer of the vector to store the result
void box_support_function(const void* data, const Vector3* direction, Vector3* output);

/// @brief Bounding Box Calculator function for the box primitive.
///
/// Calculates the AABB fully containing the (optionally rotated) box.
/// @param data pointer to the physics_object holding the box collider
/// @param rotation pointer to the physics_object rotation Quaternion (may be NULL)
/// @param box  pointer to the output AABB
void box_bounding_box(const void* data, const Quaternion* rotation, AABB* box);

/// @brief Inertia Tensor function for the box primitive.
///
/// Calculates the diagonal vector of the local inertia 
/// @param data pointer to the physics_object holding the box collider
/// @param out pointer to the 3d vector representing the inertia tensor diagonal Ixx, Iyy, Izz
void box_inertia_tensor(void* data, Vector3* out);


// Predefined Box Collider Definition
#define BOX_COLLIDER(x, y, z)          \
    .gjk_support_function = box_support_function, \
    .bounding_box_calculator = box_bounding_box,  \
    .inertia_calculator = box_inertia_tensor,     \
    .shape_type = COLLISION_SHAPE_BOX, \
    .shape_data = {                          \
        .box = {                       \
            .half_size = {{x, y, z}},    \
        },                             \
    }

#endif