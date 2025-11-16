#include "mesh_collider.h"

#include <malloc.h>
#include <assert.h>
#include <string.h>
#include <libdragon.h>


// CMSH
#define EXPECTED_HEADER 0x434D5348

void mesh_collider_load_test(struct mesh_collider* into){
    int vertex_count = 8;
    int triangle_count = 10;
    Vector3 vertices[] = {
        {{-40.0f, 0.0f, -40.0f}},
        {{40.0f, 0.0f, -40.0f}},
        {{-40.0f, 0.0f, 40.0f}},
        {{40.0f, 0.0f, 40.0f}},
        {{-40.0f, 6.0f, -40.0f}},
        {{40.0f, 6.0f, -40.0f}},
        {{-40.0f, 6.0f, 40.0f}},
        {{40.0f, 6.0f, 40.0f}},

    };

    into->vertices = malloc(sizeof(Vector3) * vertex_count);
    memcpy(into->vertices, vertices, sizeof(Vector3) * vertex_count);

    struct mesh_triangle_indices triangles[triangle_count];
    Vector3 normals[triangle_count];
    triangles[0].indices[0] = 0;
    triangles[0].indices[1] = 1;
    triangles[0].indices[2] = 2;
    normals[0] = (Vector3){{0, 1, 0}};

    triangles[1].indices[0] = 1;
    triangles[1].indices[1] = 3;
    triangles[1].indices[2] = 2;
    normals[1] = (Vector3){{0, 1, 0}};

    triangles[2].indices[0] = 2;
    triangles[2].indices[1] = 4;
    triangles[2].indices[2] = 0;
    normals[2] = (Vector3){{1, 0, 0}};

    triangles[3].indices[0] = 2;
    triangles[3].indices[1] = 6;
    triangles[3].indices[2] = 4;
    normals[3] = (Vector3){{1, 0, 0}};

    triangles[4].indices[0] = 2;
    triangles[4].indices[1] = 7;
    triangles[4].indices[2] = 6;
    normals[4] = (Vector3){{0, 0, -1}};

    triangles[5].indices[0] = 2;
    triangles[5].indices[1] = 3;
    triangles[5].indices[2] = 7;
    normals[5] = (Vector3){{0, 0, -1}};

    triangles[6].indices[0] = 3;
    triangles[6].indices[1] = 5;
    triangles[6].indices[2] = 7;
    normals[6] = (Vector3){{-1, 0, 0}};

    triangles[7].indices[0] = 1;
    triangles[7].indices[1] = 5;
    triangles[7].indices[2] = 3;
    normals[7] = (Vector3){{-1, 0, 0}};

    triangles[8].indices[0] = 1;
    triangles[8].indices[1] = 4;
    triangles[8].indices[2] = 5;
    normals[8] = (Vector3){{0, 0, 1}};

    triangles[9].indices[0] = 0;
    triangles[9].indices[1] = 4;
    triangles[9].indices[2] = 1;
    normals[9] = (Vector3){{0, 0, 1}};
    into->triangles = malloc(sizeof(struct mesh_triangle_indices) * triangle_count);
    memcpy(into->triangles, triangles, sizeof(struct mesh_triangle_indices) * triangle_count);

    into->normals = malloc(sizeof(Vector3) * triangle_count);
    memcpy(into->normals, normals, sizeof(Vector3) * triangle_count);

    into->triangle_count = triangle_count;

    AABB_tree_init(&into->aabbtree, (2 * triangle_count) + 1);
    AABB triangleAABB;
    for (int i = 0; i < triangle_count; i++)
    {
        struct mesh_triangle_indices* triangle = &into->triangles[i];
        Vector3* v0 = &into->vertices[triangle->indices[0]];
        Vector3* v1 = &into->vertices[triangle->indices[1]];
        Vector3* v2 = &into->vertices[triangle->indices[2]];

        triangleAABB = AABBFromTriangle(v0, v1, v2);

        AABB_tree_create_node(&into->aabbtree, triangleAABB, (void *)i);
    }
}

void mesh_collider_load(struct mesh_collider* into, const char* filename, float scale, Vector3* offset) {
    int header;
    FILE *file = asset_fopen(filename, NULL);
    fread(&header, 1, 4, file);
    assert(header == EXPECTED_HEADER);

    uint16_t vertex_count;
    fread(&vertex_count, 2, 1, file);
    into->vertex_count = vertex_count;

    into->vertices = malloc(sizeof(Vector3) * vertex_count);
    fread(into->vertices, sizeof(Vector3), vertex_count, file);

    for (int i = 0; i < vertex_count; i++)
    {

        Vector3* vert = &into->vertices[i];

        vert->x *= scale;
        vert->y *= scale;
        vert->z *= scale;
    }

    uint16_t triangle_count;
    fread(&triangle_count, 2, 1, file);
    into->triangle_count = triangle_count;

    into->triangles = malloc(sizeof(struct mesh_triangle_indices) * triangle_count);
    fread(into->triangles, sizeof(struct mesh_triangle_indices), triangle_count, file);

    into->normals = malloc(sizeof(Vector3) * triangle_count);
    fread(into->normals, sizeof(Vector3), triangle_count, file);
    fclose(file);

    AABB_tree_init(&into->aabbtree, (2 * triangle_count) + 1);
    AABB triangleAABB;
    for (int i = 0; i < triangle_count; i++)
    {

        struct mesh_triangle_indices* triangle = &into->triangles[i];
        
        Vector3* v0 = &into->vertices[triangle->indices[0]];
        Vector3* v1 = &into->vertices[triangle->indices[1]];
        Vector3* v2 = &into->vertices[triangle->indices[2]];

        triangleAABB = AABBFromTriangle(v0, v1, v2);

        AABB_tree_create_node(&into->aabbtree, triangleAABB, (void *)i);
    }
}

void mesh_collider_release(struct mesh_collider* mesh){
    free(mesh->vertices);
    free(mesh->triangles);
    AABB_tree_free(&mesh->aabbtree);
}