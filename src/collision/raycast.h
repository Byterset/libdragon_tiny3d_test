#ifndef __RAY_H__
#define __RAY_H__

#include "../math/vector3.h"
#include "../math/transform.h"
#include "../math/mathf.h"
#include "../entity/entity_id.h"
#include <stdbool.h>
#include <stdint.h>
#include <libdragon.h>

#define RAYCAST_MAX_DISTANCE 2000.0f // Maximum distance that is accepted for raycasts
#define RAYCAST_MAX_OBJECT_TESTS 10 // The maximum number of object candidates that will be tested against in the collision scene for a single raycast
#define RAYCAST_MAX_TRIANGLE_TESTS 15 // The maximum number of triangle candidates that will be tested against in the static collision scene for a single raycast

/// @brief The raycast collision scene mask is used to filter what the raycast will test against.
typedef enum raycast_collision_scene_mask {
    RAYCAST_COLLISION_SCENE_MASK_STATIC_COLLISION = 1 << 0,
    RAYCAST_COLLISION_SCENE_MASK_PHYSICS_OBJECTS = 1 << 1,
    RAYCAST_COLLISION_SCENE_MASK_ALL = 0xFF
} raycast_collision_scene_mask;

/// @brief The raycast structure represents a raycast in 3D space. Don't create this structure directly, use raycast_init instead.
typedef struct raycast {
    Vector3 origin; // The origin of the ray
    Vector3 dir; // The direction of the ray (will be normalized on initialization)
    Vector3 _invDir; // The inverse direction of the ray, precomputed for repeated testing against BVH nodes
    float maxDistance; // The maximum distance the ray will travel, limited to 1000.0f, cannot be negative
    raycast_collision_scene_mask mask; // Determines what the ray will test against (static collision, physics objects, etc.)
    uint16_t collision_layer_filter; // Filter for collision layers, objects that match this filter will be ignored
    bool interact_trigger; // If true, the ray will test for hits with trigger colliders
} raycast;

/// @brief The raycast hit structure represents the result of a raycast.
typedef struct raycast_hit {
    bool did_hit;
    Vector3 point; // The impact point where the ray intersects the object or surface
    Vector3 normal; // The normal of the object or surface that was hit
    float distance; // The distance from the ray origin to the hit point
    entity_id hit_entity_id; // The entity id of the object that was hit, 0 if no object was hit or the hit was against the static collision scene
} raycast_hit;

/// @brief Initializes a raycast with the given parameters and returns a new raycast struct
/// @param origin the origin point of the ray in world space
/// @param dir the direction of the ray (will be normalized during initialization)
/// @param maxDistance the maximum distance until which hits will be considered. Must be > 0
/// @param mask decides if the ray will hit physicsObjects, static Collision or both
/// @param interact_trigger toggle if the raycast will be able to hit physicsObjects that are triggers
/// @param collision_layer_filter mask of collision layers the raycast will ignore
/// @return a new initialized raycast structure
raycast raycast_init(Vector3 origin, Vector3 dir, float maxDistance, raycast_collision_scene_mask mask, bool interact_trigger, uint16_t collision_layer_filter);

/// @brief Applies a given transform to a raycast translating the origin of the ray and rotating the direction
/// @param transform 
/// @param ray 
/// @param output 
void raycast_transform(Transform* transform, raycast* ray, raycast* output);


float raycast_calc_distance_to_point(raycast* ray, Vector3* point);

/// @brief cast a ray into the existing collision scene and return true if an object or static collision triangle fitting
/// the settings of the raycast is hit.
///
/// The hit object will contain the raycast hit information of the intersection with the least distance
/// @param ray pointer to the ray to be cast
/// @param hit pointer to the resulting hit object
/// @return true if the raycast has hit anything mathing the mask & filter, false otherwise
bool raycast_cast(raycast* ray, raycast_hit* hit);

#endif