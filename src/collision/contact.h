#ifndef __COLLISION_CONTACT_H__
#define __COLLISION_CONTACT_H__

#include "../math/vector3.h"
#include "../entity/entity_id.h"
#include "physics_object.h"

#define MAX_CONTACT_POINTS_PER_PAIR 4

typedef struct contact_constraint contact_constraint;
typedef struct contact contact;
typedef uint32_t contact_pair_id; //unique combination of two entity ids (enity_id is uint16_t), must be double size of entity_id
typedef struct physics_object physics_object;


/// @brief contact struct containing information about a collision
typedef struct contact {
    contact* next; // pointer to the next contact in a list
    contact_constraint* constraint; // pointer to the shared constraint data
    physics_object* other_object; // entity_id of the object that was collided with
} contact;


/// @brief Single contact point data within a contact constraint
typedef struct contact_point {
    Vector3 point; // the 3D position in world space of the contact point
    Vector3 contactA; // contact point on surface A (world space)
    Vector3 contactB; // contact point on surface B (world space)
    Vector3 localPointA; // contact point on surface A (local space)
    Vector3 localPointB; // contact point on surface B (local space)
    float penetration; // depth of penetration for this point
    bool active; // whether this point was updated/validated this frame

    // Cached data for warm starting and iterative solving (per point)
    float accumulated_normal_impulse; // accumulated normal impulse for warm starting
    float accumulated_tangent_impulse_u; // accumulated tangent impulse for friction (first tangent direction)
    float accumulated_tangent_impulse_v; // accumulated tangent impulse for friction (second tangent direction)
    float normal_mass; // cached effective mass for normal direction (1/denominator)
    float tangent_mass_u; // cached effective mass for first tangent direction
    float tangent_mass_v; // cached effective mass for second tangent direction
    float velocity_bias; // velocity bias for restitution
    Vector3 a_to_contact; // contact point relative to A's center of mass
    Vector3 b_to_contact; // contact point relative to B's center of mass
} contact_point;


/// @brief contact constraint containing multiple contact points for a pair of objects
typedef struct contact_constraint {
    contact_pair_id pid; // unique ID for this contact pair (combination of both entity IDs)
    physics_object* objectA; // first object in the contact pair
    physics_object* objectB; // second object in the contact pair
    
    // Shared data for all contact points in this pair
    Vector3 normal; // the collision normal pointing from B toward A (shared across points)
    Vector3 tangent_u; // first tangent direction for friction (shared)
    Vector3 tangent_v; // second tangent direction for friction (shared)

    // Material properties (shared)
    float combined_friction;
    float combined_bounce;

    // Flags
    bool is_active; // was this contact found this frame?
    bool is_trigger; // is this a trigger contact (no resolution)?

    // Optimization: Linked list of constraints with the same PID
    int next_same_pid_index;

    // Multiple contact points for this pair
    contact_point points[MAX_CONTACT_POINTS_PER_PAIR];
    int point_count; // number of active contact points (1-4 typically)
} contact_constraint;

/// @brief Create a unique contact pair id from two entity ids
/// @param a 
/// @param b 
/// @return uint32_t pid
inline contact_pair_id contact_pair_id_get(entity_id a_id, entity_id b_id){
    if (a_id < b_id)
        return ((contact_pair_id)a_id << (sizeof(entity_id) * __CHAR_BIT__)) | b_id;
    else
        return ((contact_pair_id)b_id << (sizeof(entity_id) * __CHAR_BIT__)) | a_id;
}

#endif
