#ifndef __COLLISION_COLLIDE_SWEPT_H__
#define __COLLISION_COLLIDE_SWEPT_H__

#include "physics_object.h"
#include "mesh_collider.h"
#include "epa.h"

/// @brief Data structure for swept collision detection against a mesh.
struct object_mesh_collide_data {
    Vector3* prev_pos;
    struct mesh_collider* mesh;
    physics_object* object;
    struct EpaResult hit_result;
};

/// @brief Performs a swept collision check between a physics object and a static mesh.
/// @param object The physics object to check.
/// @param mesh The static mesh collider.
/// @param prev_pos The previous position of the object (start of the sweep).
/// @return true if a collision occurred, false otherwise.
bool collide_object_to_mesh_swept(physics_object* object, struct mesh_collider* mesh, Vector3* prev_pos);

#endif