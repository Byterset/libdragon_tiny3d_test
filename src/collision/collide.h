#ifndef __COLLISION_COLLIDE_H__
#define __COLLISION_COLLIDE_H__

#include "mesh_collider.h"
#include "physics_object.h"
#include "epa.h"

void collide_object_to_mesh(struct physics_object* object, struct mesh_collider* mesh);
void collide_object_to_object(struct physics_object* a, struct physics_object* b);

void correct_velocity(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce);
void correct_overlap(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce);

void collide_add_contact(struct physics_object* object, struct EpaResult* result);

#endif