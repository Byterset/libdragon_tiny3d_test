#ifndef __COLLISION_MESH_COLLIDER_H__
#define __COLLISION_MESH_COLLIDER_H__

#include <stdint.h>
#include <stdbool.h>

#include "../math/vector3.h"
#include "../math/aabb.h"
#include "aabbtree.h"
#include "../render/defs.h"

struct mesh_triangle_indices {
    uint16_t indices[3];
};

struct mesh_collider {
    struct AABBTree aabbtree;
    Vector3* vertices;
    struct mesh_triangle_indices* triangles;
    Vector3* normals;
    uint16_t triangle_count;
    uint16_t vertex_count;
    Vector3* offset;
    float scale;
};

struct mesh_triangle {
    Vector3* vertices;
    Vector3 normal;
    struct mesh_triangle_indices triangle;
};


void mesh_triangle_gjk_support_function(void* data, Vector3* direction, Vector3* output);
float mesh_triangle_comparePoint(struct mesh_triangle *triangle, Vector3 *point);

#endif