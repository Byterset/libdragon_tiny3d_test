#ifndef __COLLISION_COLLIDE_H__
#define __COLLISION_COLLIDE_H__

#include "mesh_collider.h"
#include "raycast.h"
#include "physics_object.h"
#include "epa.h"



/// @brief Corrects the velocities of colliding objects. This can affect both the linear and angular velocities of an object
/// depending on if it has any constraints or no rotation reference.
///
/// This will also correct velocities for collisions against static mesh_colliders. Here one of the input object pointers will
/// be NULL and act as a collision against an object with infinite mass.
/// @param a pointer to colliding physics_object a
/// @param b pointer to colliding physics_object b
/// @param result the result of the EPA, containing collision normal, contact points & penetration depth
/// @param friction the combined friction of the objects
/// @param bounce the combined bounce of the objects
void correct_velocity(physics_object* a, physics_object* b, const struct EpaResult* result, float friction, float bounce);


/// @brief Attempts to create a new contact in the collision_scene and add it to the physics_objects list of active contacts.
///
/// @note It is important to know in which order the objects were tested during the EPA so the correct contact point of the result gets added.
/// @param object the pointer of the object to add the contact to
/// @param result the result of the EPA, containing the calculated contact points
/// @param is_B flag if this object was object B during EPA - EpaResult will contain both contactA and contactB
/// @param other_id the entity_id of the object that was collided against
void collide_add_contact(physics_object* object, contact_constraint* constraint, physics_object* other_object);


// -------- NEW: DETECTION-ONLY FUNCTIONS FOR ITERATIVE SOLVER --------

/// @brief Detects collision between two physics objects and stores the contact in the constraint cache.
/// Does NOT resolve the collision - that happens later in the solver phases.
/// @param a physics object a
/// @param b physics object b
void detect_contact_object_to_object(physics_object* a, physics_object* b);

/// @brief Detects collisions between a physics object and a static mesh collider, storing contacts in the constraint cache.
/// Does NOT resolve collisions - that happens later in the solver phases.
/// @param object the object to detect collisions for
/// @param mesh the static mesh collider
void detect_contacts_object_to_mesh(physics_object* object, const struct mesh_collider* mesh);

/// @brief Detects collision between a physics object and a single triangle from a mesh.
/// @param object the physics object
/// @param mesh the mesh collider
/// @param triangle_index the index of the triangle to check
/// @return true if collision was detected and cached
bool detect_contact_object_to_triangle(physics_object* object, const struct mesh_collider* mesh, int triangle_index);

/// @brief Stores a detected contact in the global constraint cache for later solving.
/// @param entity_a first entity ID (0 for static mesh)
/// @param entity_b second entity ID
/// @param result EPA result containing contact information
/// @param combined_friction combined friction coefficient
/// @param combined_bounce combined bounce coefficient
/// @param is_trigger whether this is a trigger contact
contact_constraint* cache_contact_constraint(physics_object* objectA, physics_object* objectB, const struct EpaResult* result,
                               float combined_friction, float combined_bounce, bool is_trigger);

#endif