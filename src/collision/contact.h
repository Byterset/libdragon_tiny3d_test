#ifndef __COLLISION_CONTACT_H__
#define __COLLISION_CONTACT_H__

#include "../math/vector3.h"
#include "../entity/entity_id.h"
#include "physics_object.h"

#define MAX_ACTIVE_CONTACTS 128

typedef struct contact contact;
typedef uint32_t contact_id; //unique combination of two entity ids (enity_id is uint16_t)
typedef struct physics_object physics_object;
/// @brief contact struct containing information about a collision
typedef struct contact {
    contact* next; // pointer to the next contact in a list
    Vector3 point; // the 3D position in world space of the contact point on the surface of an object
    Vector3 normal; // the collision normal pointing away from the object that was collided with
    entity_id other_object; // entity_id of the object that was collided with
} contact;

/// @brief contact constraint point containing cached solver data for iterative resolution
typedef struct contact_constraint {
    contact_id id; // unique ID for this contact pair (combination of both entity IDs)
    physics_object* objectA; // first object in the contact pair
    physics_object* objectB; // second object in the contact pair
    Vector3 point; // the 3D position in world space of the contact point
    Vector3 normal; // the collision normal pointing from B toward A
    Vector3 contactA; // contact point on surface A
    Vector3 contactB; // contact point on surface B
    float penetration; // depth of penetration

    // Cached data for warm starting and iterative solving
    float accumulated_normal_impulse; // accumulated normal impulse for warm starting
    float accumulated_tangent_impulse_u; // accumulated tangent impulse for friction (first tangent direction)
    float accumulated_tangent_impulse_v; // accumulated tangent impulse for friction (second tangent direction)
    float normal_mass; // cached effective mass for normal direction (1/denominator)
    float tangent_mass_u; // cached effective mass for first tangent direction
    float tangent_mass_v; // cached effective mass for second tangent direction
    Vector3 tangent_u; // first tangent direction for friction
    Vector3 tangent_v; // second tangent direction for friction
    Vector3 rA; // contact point relative to A's center of mass
    Vector3 rB; // contact point relative to B's center of mass

    // Material properties
    float combined_friction;
    float combined_bounce;

    // Flags
    bool is_active; // was this contact found this frame?
    bool is_trigger; // is this a trigger contact (no resolution)?
} contact_constraint;

inline contact_id get_contact_id(entity_id a, entity_id b){
    if (a < b)
        return ((uint32_t)a << 16) | b;
    else
        return ((uint32_t)b << 16) | a;
}


#endif