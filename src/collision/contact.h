#ifndef __COLLISION_CONTACT_H__
#define __COLLISION_CONTACT_H__

#include "../math/vector3.h"
#include "../entity/entity_id.h"

struct physics_object;

// Old single-point contact structure (kept for backward compatibility during transition)
struct contact {
    struct contact* next;
    Vector3 point;
    Vector3 normal;
    entity_id other_object;
};

// Maximum contact points per manifold (box-box can have up to 4)
#define MAX_MANIFOLD_POINTS 4

// Individual contact point within a manifold
struct contact_point {
    Vector3 localPointA;        // Contact point in object A's local space
    Vector3 localPointB;        // Contact point in object B's local space
    Vector3 worldPoint;         // Contact point in world space
    Vector3 normal;             // Contact normal (points from A to B)
    float penetration;          // Penetration depth
    float normalImpulse;        // Accumulated normal impulse (for warm starting)
    float tangentImpulse[2];    // Accumulated friction impulses (for warm starting)
    float distance;             // Signed distance (for contact matching)
    int lifetime;               // Number of frames this contact has persisted
};

// Contact manifold - represents all contact points between two objects
struct contact_manifold {
    struct contact_manifold* next;
    struct physics_object* objectA;
    struct physics_object* objectB;
    entity_id entityA;
    entity_id entityB;
    int numPoints;
    struct contact_point points[MAX_MANIFOLD_POINTS];
    float friction;             // Cached friction coefficient
    float restitution;          // Cached restitution coefficient
    int manifoldLifetime;       // How long this manifold has existed
};

#endif