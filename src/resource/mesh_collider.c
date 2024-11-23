#include "mesh_collider.h"

#include <malloc.h>
#include <assert.h>
#include <string.h>
#include <libdragon.h>


// CMSH
#define EXPECTED_HEADER 0x434D5348

#ifdef DEBUG_COLLIDERS_RAYLIB
Raylib_Mesh mesh_collider_generate_raylib_mesh(struct mesh_collider* mesh, int vertex_count, int triangle_count) {
    Raylib_Mesh raylib_mesh = {0};

    raylib_mesh.vertexCount = vertex_count;
    raylib_mesh.triangleCount = triangle_count;
    raylib_mesh.vertices = malloc(sizeof(float) * vertex_count * 3);
    raylib_mesh.texcoords = malloc(sizeof(float) * vertex_count * 2);

    raylib_mesh.normals = malloc(sizeof(float) * vertex_count * 3);

    raylib_mesh.indices = malloc(sizeof(unsigned short) * triangle_count * 3);

    for (int i = 0; i < vertex_count; i++)
    {
        raylib_mesh.vertices[i * 3] = mesh->vertices[i].x;
        raylib_mesh.vertices[i * 3 + 1] = mesh->vertices[i].y;
        raylib_mesh.vertices[i * 3 + 2] = mesh->vertices[i].z;
        raylib_mesh.normals[i * 3] = 0;
        raylib_mesh.normals[i * 3 + 1] = 1;
        raylib_mesh.normals[i * 3 + 2] = 0;
        raylib_mesh.texcoords[i * 2] = 0;
        raylib_mesh.texcoords[i * 2 + 1] = 0;
    }

    for (int i = 0; i < triangle_count; i++)
    {
        raylib_mesh.indices[i * 3] = mesh->triangles[i].indices[2];
        raylib_mesh.indices[i * 3 + 1] = mesh->triangles[i].indices[1];
        raylib_mesh.indices[i * 3 + 2] = mesh->triangles[i].indices[0];
    }

    UploadMesh(&raylib_mesh, false);
    return raylib_mesh;
}
#endif

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

    AABBTree_create(&into->aabbtree, (2 * triangle_count) + 1);
    AABB triangleAABB;
    for (int i = 0; i < triangle_count; i++)
    {
        struct mesh_triangle_indices* triangle = &into->triangles[i];
        Vector3* v0 = &into->vertices[triangle->indices[0]];
        Vector3* v1 = &into->vertices[triangle->indices[1]];
        Vector3* v2 = &into->vertices[triangle->indices[2]];

        triangleAABB = AABBFromTriangle(v0, v1, v2);

        AABBTreeNode_createNode(&into->aabbtree, triangleAABB, (void *)i);
    }
#ifdef DEBUG_COLLIDERS_RAYLIB
    into->raylib_mesh_model = LoadModelFromMesh(mesh_collider_generate_raylib_mesh((struct mesh_collider*)into, vertex_count, triangle_count));
    Image white = GenImageColor(1, 1, WHITE);
    Texture2D texture = LoadTextureFromImage(white);

    into->raylib_mesh_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    #endif
}

void mesh_collider_load(struct mesh_collider* into, const char* filename) {
    int header;
    FILE *file = asset_fopen(filename, NULL);
    fread(&header, 1, 4, file);
    assert(header == EXPECTED_HEADER);

    uint16_t vertex_count;
    fread(&vertex_count, 2, 1, file);
    into->vertex_count = vertex_count;
    debugf("Vertex count: %d\n", vertex_count);

    into->vertices = malloc(sizeof(Vector3) * vertex_count);
    fread(into->vertices, sizeof(Vector3), vertex_count, file);

    uint16_t triangle_count;
    fread(&triangle_count, 2, 1, file);
    into->triangle_count = triangle_count;
    debugf("Triangle count: %d\n", triangle_count);

    into->triangles = malloc(sizeof(struct mesh_triangle_indices) * triangle_count);
    fread(into->triangles, sizeof(struct mesh_triangle_indices), triangle_count, file);

    into->normals = malloc(sizeof(Vector3) * triangle_count);
    fread(into->normals, sizeof(Vector3), triangle_count, file);
    fclose(file);

    AABBTree_create(&into->aabbtree, (2 * triangle_count) + 1);
    AABB triangleAABB;
    for (int i = 0; i < triangle_count; i++)
    {

        struct mesh_triangle_indices* triangle = &into->triangles[i];
        if (i == 0)
        {
            debugf("Triangle %d: %.2f, %.2f, %.2f\n", i, into->vertices[triangle->indices[0]].x, into->vertices[triangle->indices[0]].y, into->vertices[triangle->indices[0]].z);
        }
        Vector3* v0 = &into->vertices[triangle->indices[0]];
        Vector3* v1 = &into->vertices[triangle->indices[1]];
        Vector3* v2 = &into->vertices[triangle->indices[2]];

        triangleAABB = AABBFromTriangle(v0, v1, v2);

        AABBTreeNode_createNode(&into->aabbtree, triangleAABB, (void *)i);
    }
#ifdef DEBUG_COLLIDERS_RAYLIB
    into->raylib_mesh_model = LoadModelFromMesh(mesh_collider_generate_raylib_mesh((struct mesh_collider*)into, vertex_count, triangle_count));
    Image white = GenImageColor(1, 1, WHITE);
    Texture2D texture = LoadTextureFromImage(white);

    into->raylib_mesh_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    #endif

}

void mesh_collider_release(struct mesh_collider* mesh){
    free(mesh->vertices);
    free(mesh->triangles);
    AABBTree_free(&mesh->aabbtree);
#ifdef DEBUG_COLLIDERS_RAYLIB
    UnloadTexture(mesh->raylib_mesh_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture);
    UnloadModel(mesh->raylib_mesh_model);
    #endif
}

