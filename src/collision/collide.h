#ifndef __COLLISION_COLLIDE_H__
#define __COLLISION_COLLIDE_H__

#include "mesh_collider.h"
#include "raycast.h"
#include "physics_object.h"
#include "epa.h"

void collide_object_to_mesh(physics_object* object, struct mesh_collider* mesh);
void collide_object_to_object(physics_object* a, physics_object* b);

void correct_velocity(physics_object* a, physics_object* b, struct EpaResult* result, float friction, float bounce);
void correct_overlap(physics_object* a, physics_object* b, struct EpaResult* result);

void collide_add_contact(physics_object* object, struct EpaResult* result, bool is_B, entity_id other_id);

#endif