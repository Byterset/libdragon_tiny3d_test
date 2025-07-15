#ifndef __COLLISION_SHAPE_RAY_SHAPE_INTERSECTION_H__
#define __COLLISION_SHAPE_RAY_SHAPE_INTERSECTION_H__

#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../raycast.h"
#include "../physics_object.h"

/**
 * @brief Tests for intersection between a ray and a sphere
 * 
 * @param ray The ray to test
 * @param center The center of the sphere
 * @param radius The radius of the sphere
 * @param hit The hit result to fill if there's an intersection
 * @param entity_id The entity ID to set in the hit result
 * @return true if there's an intersection, false otherwise
 */
bool ray_sphere_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    raycast_hit* hit,
    entity_id entity_id
);

/**
 * @brief Tests for intersection between a ray and a box
 * 
 * @param ray The ray to test
 * @param center The center of the box
 * @param half_size The half-size of the box
 * @param rotation The rotation of the box
 * @param hit The hit result to fill if there's an intersection
 * @param entity_id The entity ID to set in the hit result
 * @return true if there's an intersection, false otherwise
 */
bool ray_box_intersection(
    raycast* ray, 
    Vector3* center, 
    Vector3* half_size, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
);

/**
 * @brief Tests for intersection between a ray and a capsule
 * 
 * @param ray The ray to test
 * @param center The center of the capsule
 * @param radius The radius of the capsule
 * @param half_height The half-height of the capsule (inner segment)
 * @param rotation The rotation of the capsule
 * @param hit The hit result to fill if there's an intersection
 * @param entity_id The entity ID to set in the hit result
 * @return true if there's an intersection, false otherwise
 */
bool ray_capsule_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    float half_height, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
);

/**
 * @brief Tests for intersection between a ray and a cylinder
 * 
 * @param ray The ray to test
 * @param center The center of the cylinder
 * @param radius The radius of the cylinder
 * @param half_height The half-height of the cylinder
 * @param rotation The rotation of the cylinder
 * @param hit The hit result to fill if there's an intersection
 * @param entity_id The entity ID to set in the hit result
 * @return true if there's an intersection, false otherwise
 */
bool ray_cylinder_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    float half_height, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
);

/**
 * @brief Tests for intersection between a ray and a cone
 * 
 * @param ray The ray to test
 * @param center The center of the cone
 * @param radius The radius of the cone
 * @param half_height The half-height of the cone
 * @param rotation The rotation of the cone
 * @param hit The hit result to fill if there's an intersection
 * @param entity_id The entity ID to set in the hit result
 * @return true if there's an intersection, false otherwise
 */
bool ray_cone_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    float half_height, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
);

/**
 * @brief Tests for intersection between a ray and a physics object
 * 
 * @param ray The ray to test
 * @param object The physics object to test
 * @param hit The hit result to fill if there's an intersection
 * @return true if there's an intersection, false otherwise
 */
bool ray_physics_object_intersection(
    raycast* ray, 
    struct physics_object* object, 
    raycast_hit* hit
);

#endif // __COLLISION_SHAPE_RAY_SHAPE_INTERSECTION_H__
