#ifndef __COLLISION_CONTACT_H__
#define __COLLISION_CONTACT_H__

#include "../math/vector3.h"
#include "../entity/entity_id.h"

#define MAX_ACTIVE_CONTACTS 128

typedef struct contact contact;

/// @brief contact struct containing information about a collision
typedef struct contact {
    contact* next; // pointer to the next contact in a list
    Vector3 point; // the 3D position in world space of the contact point on the surface of an object
    Vector3 normal; // the collision normal pointing away from the object that was collided with
    entity_id other_object; // entity_id of the object that was collided with
} contact;

#endif