#ifndef __COLLISION_SHAPE_PYRAMID_H__
#define __COLLISION_SHAPE_PYRAMID_H__

#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../../math/aabb.h"

/// @brief GJK Support function (see gjk_support_function) for the pyramid primitive.
///
/// Will return the point on the pyramid boundary furthest in the input direction.
/// @param data pointer to the physics_object holding the pyramid collider
/// @param direction input direction (is expected to be normalized and rotated if the object has rotation)
/// @param output the pointer of the vector to store the result
void pyramid_support_function(const void* data, const Vector3* direction, Vector3* output);

/// @brief Bounding Box Calculator function for the pyramid primitive.
///
/// Calculates the AABB fully containing the (optionally rotated) pyramid.
/// @param data pointer to the physics_object holding the pyramid collider
/// @param rotation pointer to the physics_object rotation Quaternion (may be NULL)
/// @param box  pointer to the output AABB
void pyramid_bounding_box(const void* data, const Quaternion* rotation, AABB* box);

/// @brief Inertia Tensor function for the pyramid primitive.
///
/// Calculates the diagonal vector of the local inertia 
/// @param data pointer to the physics_object holding the pyramid collider
/// @param out pointer to the 3d vector representing the inertia tensor diagonal Ixx, Iyy, Izz
void pyramid_inertia_tensor(void* data, Vector3* out);


// Predefined Box Collider Definition
#define PYRAMID_COLLIDER(hwx, hwz, hh)          \
    .gjk_support_function = pyramid_support_function, \
    .bounding_box_calculator = pyramid_bounding_box,  \
    .inertia_calculator = pyramid_inertia_tensor,     \
    .shape_type = COLLISION_SHAPE_PYRAMID, \
    .shape_data = {                          \
        .pyramid = {                       \
            .base_half_widths = {{hwx, hwz}},    \
            .half_height = hh, \
        },                             \
    }

#endif