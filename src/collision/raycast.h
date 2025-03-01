#ifndef __RAY_H__
#define __RAY_H__

#include "../math/vector3.h"
#include "../math/transform.h"
#include "../math/mathf.h"
#include "../entity/entity_id.h"
#include <stdbool.h>

/// @brief The raycast collision scene mask is used to filter what the raycast will test against.
typedef enum raycast_collision_scene_mask {
    RAYCAST_COLLISION_SCENE_MASK_STATIC_COLLISION = 1 << 0,
    RAYCAST_COLLISION_SCENE_MASK_PHYSICS_OBJECTS = 1 << 1,
    RAYCAST_COLLISION_SCENE_MASK_ALL = 0xFF
} raycast_collision_scene_mask;

/// @brief The raycast structure represents a raycast in 3D space. Don't create this structure directly, use raycast_init instead.
typedef struct raycast {
    Vector3 origin;
    Vector3 dir;
    Vector3 _invDir;
    float maxDistance;
    raycast_collision_scene_mask mask;
    bool interact_trigger;
} raycast;

/// @brief The raycast hit structure represents the result of a raycast.
typedef struct raycast_hit {
    Vector3 point; // The impact point where the ray intersects the object or surface
    Vector3 normal; // The normal of the object or surface that was hit
    float distance; // The distance from the ray origin to the hit point
    entity_id hit_entity_id; // The entity id of the object that was hit, 0 if no object was hit or the hit was against the static collision scene
} raycast_hit;

raycast raycast_init(Vector3 origin, Vector3 dir, float maxDistance, raycast_collision_scene_mask mask, bool interact_trigger);

void raycast_transform(Transform* transform, raycast* ray, raycast* output);

float raycast_calc_distance_to_point(raycast* ray, Vector3* point);

int raycast_cast(raycast* ray, raycast_hit* hit, raycast_collision_scene_mask mask);

#endif