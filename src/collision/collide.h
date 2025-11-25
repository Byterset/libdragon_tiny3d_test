#ifndef __COLLISION_COLLIDE_H__
#define __COLLISION_COLLIDE_H__

#include "mesh_collider.h"
#include "raycast.h"
#include "physics_object.h"
#include "epa.h"

/// @brief Adds a contact constraint to the physics object's active contact list.
/// @param object The physics object to add the contact to.
/// @param constraint The contact constraint to add.
/// @param other_object The other physics object involved in the collision.
void collide_add_contact(physics_object* object, contact_constraint* constraint, physics_object* other_object);

/// @brief Applies velocity corrections to an object based on a collision result.
/// @param object The object to correct.
/// @param result The EPA result containing collision normal and penetration.
/// @param friction The combined friction coefficient.
/// @param bounce The combined bounce coefficient.
void collide_correct_velocity(physics_object* object, const struct EpaResult* result, float friction, float bounce);

// ============================================================================
// DETECTION-ONLY FUNCTIONS (Iterative Solver)
// ============================================================================

/// @brief Detects collision between two physics objects and caches the contact constraint.
/// @param a The first physics object.
/// @param b The second physics object.
void collide_detect_object_to_object(physics_object* a, physics_object* b);

/// @brief Detects collisions between a physics object and a static mesh collider.
/// @param object The physics object.
/// @param mesh The static mesh collider.
void collide_detect_object_to_mesh(physics_object* object, const struct mesh_collider* mesh);

/// @brief Detects collision between a physics object and a single mesh triangle.
/// @param object The physics object.
/// @param mesh The mesh collider containing the triangle.
/// @param triangle_index The index of the triangle in the mesh.
/// @return true if a collision was detected and cached, false otherwise.
bool collide_detect_object_to_triangle(physics_object* object, const struct mesh_collider* mesh, int triangle_index);

/// @brief Caches a detected contact constraint for later solving.
/// @param object_a The first physics object (or NULL for static mesh).
/// @param object_b The second physics object.
/// @param result The EPA result containing contact information.
/// @param combined_friction The combined friction coefficient.
/// @param combined_bounce The combined bounce coefficient.
/// @param is_trigger Whether this is a trigger interaction.
/// @return A pointer to the cached contact constraint, or NULL if cache is full.
contact_constraint *collide_cache_contact_constraint(physics_object *object_a, physics_object *object_b, const struct EpaResult *result,
                                                     float combined_friction, float combined_bounce, bool is_trigger);

#endif