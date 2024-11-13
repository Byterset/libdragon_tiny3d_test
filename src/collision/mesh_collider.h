#ifndef __COLLISION_MESH_COLLIDER_H__
#define __COLLISION_MESH_COLLIDER_H__

#include <stdint.h>
#include <stdbool.h>

#include "../math/vector3.h"
#include "../math/aabb.h"
#include "aabbtree.h"
#include "../render/defs.h"

#ifdef DEBUG_COLLIDERS_RAYLIB
#include <raylib.h>
#include <rlgl.h>
#endif

struct mesh_triangle_indices {
    uint16_t indices[3];
};

struct mesh_index_block {
    uint16_t first_index;
    uint16_t last_index;
};

struct mesh_index {
    Vector3 min;
    Vector3 stride_inv;
    struct Vector3u8 block_count;

    struct mesh_index_block* blocks;
    uint16_t* index_indices;
};

struct mesh_collider {
    struct AABBTree aabbtree;
    Vector3* vertices;
    struct mesh_triangle_indices* triangles;
    Vector3* normals;
    uint16_t triangle_count;
    uint16_t vertex_count;
#ifdef DEBUG_COLLIDERS_RAYLIB
    Raylib_Model raylib_mesh_model;
#endif
};

struct mesh_triangle {
    Vector3* vertices;
    Vector3 normal;
    struct mesh_triangle_indices triangle;
};

typedef bool (*triangle_callback)(struct mesh_index* index, void* data, int triangle_index);

void mesh_triangle_gjk_support_function(void* data, Vector3* direction, Vector3* output);
float mesh_triangle_comparePoint(struct mesh_triangle *triangle, Vector3 *point);

#endif