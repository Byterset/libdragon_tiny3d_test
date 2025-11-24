#include "collide_swept.h"

#include <math.h>
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
    Vector3* prev_pos,
    struct mesh_collider* mesh,
    physics_object* object
) {
    data->prev_pos = prev_pos;
    data->mesh = mesh;
    data->object = object;
}

bool collide_object_swept_to_triangle(void* data, int triangle_index) {
    struct object_mesh_collide_data* collide_data = (struct object_mesh_collide_data*)data;
    struct swept_physics_object swept;
    swept.object = collide_data->object;
    vector3Sub(collide_data->prev_pos, collide_data->object->position, &swept.offset);

    struct mesh_triangle triangle;
    triangle.vertices = collide_data->mesh->vertices;
    triangle.triangle = collide_data->mesh->triangles[triangle_index];
    triangle.normal = collide_data->mesh->normals[triangle_index];

    struct Simplex simplex;
    Vector3 firstDir = gRight;
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, &swept, swept_physics_object_gjk_support_function, &firstDir)) {
        return false;
    }

    struct EpaResult result;
    if (epaSolveSwept(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            &swept,
            swept_physics_object_gjk_support_function,
            collide_data->prev_pos,
            collide_data->object->position,
            &result))
    {
        collide_data->hit_result = result;
        return true;
    }

    Vector3 final_pos = *collide_data->object->position;
    *collide_data->object->position = *collide_data->prev_pos;

    if (epaSolve(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            collide_data->object,
            physics_object_gjk_support_function,
            &result))
    {
        collide_data->hit_result = result;
        return true;
    }
    *collide_data->object->position = final_pos;

    return false;
}

/**
 * @brief Handles the collision response for a physics object with a swept bounce.
 *
 * This function updates the position and velocity of a physics object when it collides
 * with another object, taking into account the bounce factor of the collision.
 *
 * @param object Pointer to the physics object that is colliding.
 * @param collide_data Pointer to the collision data containing information about the collision.
 * @param start_pos Pointer to the position of the object before the collision applies (after mover/phys update, before collision resolution).
 */
void collide_object_swept_bounce(
    physics_object* object, 
    struct object_mesh_collide_data* collide_data,
    Vector3* start_pos
) {
    // this is the new prev position when iterating
    // over mulitple swept collisions
    *collide_data->prev_pos = *object->position;

    Vector3 move_amount;
    vector3Sub(start_pos, object->position, &move_amount);

    // split the move amount due to the collision into normal and tangent components
    Vector3 move_amount_normal;
    vector3Project(&move_amount, &collide_data->hit_result.normal, &move_amount_normal);
    Vector3 move_amount_tangent;
    vector3Sub(&move_amount, &move_amount_normal, &move_amount_tangent);

    vector3Scale(&move_amount_normal, &move_amount_normal, -object->collision->bounce);


    vector3Add(object->position, &move_amount_normal, object->position);
    vector3Add(object->position, &move_amount_tangent, object->position);
    // don't include friction on a bounce
    correct_velocity(NULL, object, &collide_data->hit_result, 0.0f, object->collision->bounce);

    vector3Sub(object->position, start_pos, &move_amount);
    vector3Add(&move_amount, &object->bounding_box.min, &object->bounding_box.min);
    vector3Add(&move_amount, &object->bounding_box.max, &object->bounding_box.max);

    // Cache the contact (entity_a = NULL for static mesh)
    contact_constraint *constraint = cache_contact_constraint(NULL, object, &collide_data->hit_result, object->collision->friction, 0, false);

    // Still add to old contact list for ground detection logic
    collide_add_contact(object, constraint, NULL);

}

bool collide_object_to_mesh_swept(physics_object* object, struct mesh_collider* mesh, Vector3* prev_pos){
    struct object_mesh_collide_data collide_data;
    object_mesh_collide_data_init(&collide_data, prev_pos, mesh, object);

    Vector3 start_pos = *object->position;

    Vector3 offset;


    vector3Sub(
        object->position,
        prev_pos,
        &offset);

    Vector3 box_center;
    vector3Add(&object->bounding_box.min, &object->bounding_box.max, &box_center);
    vector3Scale(&box_center, &box_center, 0.5f);

    Vector3 center_offset_from_pos;
    vector3Sub(&box_center, object->position, &center_offset_from_pos);

    Vector3 prev_box_center;
    vector3Add(prev_pos, &center_offset_from_pos, &prev_box_center);

    Vector3 box_extent;
    vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &box_extent);
    vector3Scale(&box_extent, &box_extent, 0.5f);
    AABB prev_box;
    vector3Sub(&prev_box_center, &box_extent, &prev_box.min);
    vector3Add(&prev_box_center, &box_extent, &prev_box.max);

    // span a box from the previous position to the current position to catch all possible triangle collisions
    AABB expanded_box = AABBUnion(&prev_box, &object->bounding_box);


    int result_count = 0;
    int max_results = 128;
    node_proxy results[max_results];

    AABB_tree_query_bounds(&mesh->aabbtree, &expanded_box, results, &result_count, max_results);
    
    Vector3 original_prev_pos = *prev_pos;
    Vector3 best_hit_pos = *object->position;
    struct EpaResult best_result;
    bool any_hit = false;
    float min_dist_sq = -1.0f;

    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABB_tree_get_node_data(&mesh->aabbtree, results[j]);

        // Use a temp prev_pos for this check to avoid modifying the global state
        // until we find the best hit
        Vector3 temp_prev_pos = original_prev_pos;
        collide_data.prev_pos = &temp_prev_pos;

        if (collide_object_swept_to_triangle(&collide_data, triangle_index)) {
            Vector3 diff;
            vector3Sub(&temp_prev_pos, &original_prev_pos, &diff);
            float dist_sq = vector3Dot(&diff, &diff);
            
            // If we hit something, we want the earliest hit (smallest distance from start)
            // But if dist_sq is 0, it means we are already overlapping at the start.
            // We should prioritize non-zero distance hits (actual swept collisions) over initial overlaps?
            // Or maybe initial overlaps are more critical?
            // Actually, if we are overlapping at start, we might be stuck.
            // Let's treat all hits equally based on distance.
            
            if (!any_hit || dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_hit_pos = temp_prev_pos;
                best_result = collide_data.hit_result;
                any_hit = true;
            }
        }
    }

    if (!any_hit)
    {
        return false;
    }

    // Restore the best hit into the actual prev_pos pointer
    *prev_pos = best_hit_pos;
    collide_data.prev_pos = prev_pos;
    collide_data.hit_result = best_result;

    // Move object to the contact point so the bounce calculation is correct
    *object->position = best_hit_pos;

    collide_object_swept_bounce(object, &collide_data, &start_pos);

    return true;
}