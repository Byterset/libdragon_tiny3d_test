#ifndef __COLLISION_SHAPE_SPHERE_H__
#define __COLLISION_SHAPE_SPHERE_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

/// @brief GJK Support function (see gjk_support_function) for the sphere primitive.
///
/// Will return the point on the sphere boundary furthest in the input direction.
/// @param data pointer to the physics_object holding the sphere collider
/// @param direction input direction (is expected to be normalized and rotated if the object has rotation)
/// @param output the pointer of the vector to store the result
void sphere_support_function(const void* data, const Vector3* direction, Vector3* output);

/// @brief Bounding Box Calculator function for the sphere primitive.
///
/// Calculates the AABB fully containing the (optionally rotated) sphere.
/// @param data pointer to the physics_object holding the sphere collider
/// @param rotation pointer to the physics_object rotation Quaternion (may be NULL)
/// @param box  pointer to the output AABB
void sphere_bounding_box(const void* data, const Quaternion* rotation, AABB* box);

/// @brief Inertia Tensor function for the sphere primitive.
///
/// Calculates the diagonal vector of the local inertia 
/// @param data pointer to the physics_object holding the sphere collider
/// @param out pointer to the 3d vector representing the inertia tensor diagonal Ixx, Iyy, Izz
void sphere_inertia_tensor(void* data, Vector3* out);


// Predefined Sphere Collider Definition
#define SPHERE_COLLIDER(r)          \
    .gjk_support_function = sphere_support_function, \
    .bounding_box_calculator = sphere_bounding_box,  \
    .inertia_calculator = sphere_inertia_tensor,     \
    .shape_type = COLLISION_SHAPE_SPHERE, \
    .shape_data = {                          \
        .sphere = {                       \
            .radius = r,    \
        },                             \
    }

#endif