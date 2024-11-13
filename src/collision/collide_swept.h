#ifndef __COLLISION_COLLIDE_SWEPT_H__
#define __COLLISION_COLLIDE_SWEPT_H__

#include "physics_object.h"
#include "mesh_collider.h"
#include "epa.h"


struct object_mesh_collide_data {
    Vector3* prev_pos;
    struct mesh_collider* mesh;
    struct physics_object* object;
    struct EpaResult hit_result;
};

bool collide_object_to_mesh_swept(struct physics_object* object, struct mesh_collider* mesh, Vector3* prev_pos);

#endif