#ifndef __COLLISION_SHAPE_CONE_H__
#define __COLLISION_SHAPE_CONE_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

/// @brief GJK Support function (see gjk_support_function) for the cone primitive.
///
/// Will return the point on the cone boundary furthest in the input direction.
/// @param data pointer to the physics_object holding the cone collider
/// @param direction input direction (is expected to be normalized and rotated if the object has rotation)
/// @param output the pointer of the vector to store the result
void cone_support_function(const void* data, const Vector3* direction, Vector3* output);

/// @brief Bounding Box Calculator function for the cone primitive.
///
/// Calculates the AABB fully containing the (optionally rotated) cone.
/// @param data pointer to the physics_object holding the cone collider
/// @param rotation pointer to the physics_object rotation Quaternion (may be NULL)
/// @param box  pointer to the output AABB
void cone_bounding_box(const void* data, const Quaternion* rotation, AABB* box);

/// @brief Inertia Tensor function for the cone primitive.
///
/// Calculates the diagonal vector of the local inertia 
/// @param data pointer to the physics_object holding the cone collider
/// @param out pointer to the 3d vector representing the inertia tensor diagonal Ixx, Iyy, Izz
void cone_inertia_tensor(void* data, Vector3* out);


// Predefined Cone Collider Definition
#define CONE_COLLIDER(r, hh)          \
    .gjk_support_function = cone_support_function, \
    .bounding_box_calculator = cone_bounding_box,  \
    .inertia_calculator = cone_inertia_tensor,     \
    .shape_type = COLLISION_SHAPE_CONE, \
    .shape_data = {                          \
        .cone = {                       \
            .radius = r,    \
            .half_height = hh \
        },                             \
    }

#endif