#ifndef __COLLISION_COLLIDE_SWEPT_H__
#define __COLLISION_COLLIDE_SWEPT_H__

#include "physics_object.h"
#include "mesh_collider.h"
#include "epa.h"


struct object_mesh_collide_data {
    struct mesh_collider* mesh;
    Vector3 offset;
    physics_object* object;
    struct EpaResult hit_result;
};

bool collide_object_to_mesh_swept(physics_object* object, struct mesh_collider* mesh, Vector3 offset);
#endif