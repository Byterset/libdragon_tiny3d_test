#ifndef __RESOURCE_COLLISION_MESH_H__
#define __RESOURCE_COLLISION_MESH_H__

#include "../collision/mesh_collider.h"
#include <stdio.h>

void mesh_collider_load_test(struct mesh_collider* into);
void mesh_collider_load(struct mesh_collider* into, const char* filename, float scale, Vector3* offset);
void mesh_collider_release(struct mesh_collider* mesh);

#endif