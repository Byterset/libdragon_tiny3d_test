#ifndef __COLLISION_CONTACT_H__
#define __COLLISION_CONTACT_H__

#include "../math/vector3.h"
#include "../entity/entity_id.h"

#define MAX_ACTIVE_CONTACTS 128

typedef struct contact contact;

typedef struct contact {
    contact* next;
    Vector3 point;
    Vector3 normal;
    entity_id other_object;
} contact;

#endif