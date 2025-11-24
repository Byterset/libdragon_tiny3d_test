#include "collide_swept.h"

#include <math.h>
#include <float.h>
#include "collision_scene.h"
#include "collide.h"
#include "gjk.h"
#include "epa.h"
#include <stdio.h>

struct swept_physics_object {
    physics_object* object;
    Vector3 offset;
};

void swept_physics_object_gjk_support_function(const void* data, const Vector3* direction, Vector3* output) {
    struct swept_physics_object* obj = (struct swept_physics_object*)data;
    Vector3 norm_dir;
    vector3Normalize(direction, &norm_dir);
    physics_object_gjk_support_function(obj->object, direction, output);

    if (vector3Dot(&obj->offset, &norm_dir) > 0.0f) {
        vector3Add(output, &obj->offset, output);
    }
}

void object_mesh_collide_data_init(
    struct object_mesh_collide_data* data,
    struct mesh_collider* mesh,
    physics_object* object,
    Vector3 offset
) {
    data->mesh = mesh;
    data->object = object;
    data->offset = offset;
}

bool check_swept_collision_with_triangle(
    struct object_mesh_collide_data* collide_data,
    int triangle_index,
    Vector3* out_hit_pos,
    struct EpaResult* out_result
) {
    struct swept_physics_object swept;
    swept.object = collide_data->object;
    swept.offset = collide_data->offset;
    
    Vector3 start_pos = *collide_data->object->position;
    Vector3 projected_end;
    vector3Add(&start_pos, &collide_data->offset, &projected_end);

    struct mesh_triangle triangle;
    triangle.vertices = collide_data->mesh->vertices;
    triangle.triangle = collide_data->mesh->triangles[triangle_index];
    triangle.normal = collide_data->mesh->normals[triangle_index];

    struct Simplex simplex;
    Vector3 firstDir = gRight;
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, &swept, swept_physics_object_gjk_support_function, &firstDir)) {
        return false;
    }

    if (epaSolveSwept(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            &swept,
            swept_physics_object_gjk_support_function,
            &start_pos,
            &projected_end,
            out_result))
    {
        *out_hit_pos = start_pos;
        return true;
    }

    // Fallback: check static collision at start position if swept failed but GJK overlapped
    if (epaSolve(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            collide_data->object,
            physics_object_gjk_support_function,
            out_result))
    {
        *out_hit_pos = *collide_data->object->position;
        return true;
    }

    return false;
}


bool collide_object_to_mesh_swept(physics_object* object, struct mesh_collider* mesh, Vector3 offset){
    struct object_mesh_collide_data collide_data;
    object_mesh_collide_data_init(&collide_data, mesh, object, offset);

    Vector3 start_pos = *object->position;
    Vector3 projected_pos = *object->position;
    Vector3 proj_pos_buff;
    vector3AddToSelf(&projected_pos, &offset);
    vector3AddScaled(object->position, &offset, 1.1f, &proj_pos_buff);

    Vector3 box_extent;
    vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &box_extent);
    vector3Scale(&box_extent, &box_extent, 0.5f);

    AABB proj_box;
    vector3Sub(&proj_pos_buff, &box_extent, &proj_box.min);
    vector3Add(&proj_pos_buff, &box_extent, &proj_box.max);

    // span a box from the previous position to the current position to catch all possible triangle collisions
    AABB expanded_box = AABBUnion(&proj_box, &object->bounding_box);


    int result_count = 0;
    int max_results = 128;
    node_proxy results[max_results];

    AABB_tree_query_bounds(&mesh->aabbtree, &expanded_box, results, &result_count, max_results);

    bool did_hit = false;
    float min_dist_sq = FLT_MAX;
    Vector3 best_hit_pos = projected_pos;
    struct EpaResult best_result;

    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABB_tree_get_node_data(&mesh->aabbtree, results[j]);
        
        Vector3 hit_pos;
        struct EpaResult result;
        if (check_swept_collision_with_triangle(&collide_data, triangle_index, &hit_pos, &result)) {
            float dist_sq = vector3DistSqrd(&start_pos, &hit_pos);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_hit_pos = hit_pos;
                best_result = result;
                did_hit = true;
            }
        }
    }

    if (did_hit) {
        *object->position = best_hit_pos;
        collide_data.hit_result = best_result;
        // Cache constraint
        contact_constraint *constraint = cache_contact_constraint(NULL, object, &collide_data.hit_result, object->collision->friction, object->collision->bounce, false);
        // Add to active contacts (for gameplay logic etc)
        collide_add_contact(object, constraint, NULL);
        return true;
    } else {
        *object->position = projected_pos;
        return false;
    }
}