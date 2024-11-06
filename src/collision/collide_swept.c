#include "collide_swept.h"

#include <math.h>
#include "collision_scene.h"
#include "collide.h"
#include "gjk.h"
#include "epa.h"
#include <stdio.h>

struct swept_physics_object {
    struct physics_object* object;
    struct Vector3 offset;
};

void swept_physics_object_gjk_support_function(void* data, struct Vector3* direction, struct Vector3* output) {
    struct swept_physics_object* obj = (struct swept_physics_object*)data;
    struct Vector3 norm_dir;
    vector3Normalize(direction, &norm_dir);
    physics_object_gjk_support_function(obj->object, direction, output);

    if (vector3Dot(&obj->offset, &norm_dir) > 0.0f) {
        vector3Add(output, &obj->offset, output);
    }
}

void object_mesh_collide_data_init(
    struct object_mesh_collide_data* data,
    struct Vector3* prev_pos,
    struct mesh_collider* mesh,
    struct physics_object* object
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
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, &swept, swept_physics_object_gjk_support_function, &gRight)) {
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

    struct Vector3 final_pos = *collide_data->object->position;
    *collide_data->object->position = *collide_data->prev_pos;

    if (epaSolve(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            collide_data->object,
            physics_object_gjk_support_function,
            &result))
    {
        // struct Vector3 old_coll_center = collide_data->object->collision->collider_world_center;
        // vector3SubFromSelf(&old_coll_center, &swept.offset);
        collide_data->hit_result = result;
        // collide_data->in_front_of_triangle = mesh_triangle_comparePoint(&triangle, &old_coll_center) >= 0;
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
    struct physics_object* object, 
    struct object_mesh_collide_data* collide_data,
    struct Vector3* start_pos
) {
    // this is the new prev position when iterating
    // over mulitple swept collisions
    *collide_data->prev_pos = *object->position;

    struct Vector3 move_amount;
    vector3Sub(start_pos, object->position, &move_amount);

    // split the move amount due to the collision into normal and tangent components
    struct Vector3 move_amount_normal;
    vector3Project(&move_amount, &collide_data->hit_result.normal, &move_amount_normal);
    struct Vector3 move_amount_tangent;
    vector3Sub(&move_amount, &move_amount_normal, &move_amount_tangent);

    vector3Scale(&move_amount_normal, &move_amount_normal, -object->collision->bounce);


    vector3Add(object->position, &move_amount_normal, object->position);
    vector3Add(object->position, &move_amount_tangent, object->position);
    vector3Copy(object->position, &object->verlet_prev_position);

    // don't include friction on a bounce
    // if(collide_data->in_front_of_triangle){
        correct_velocity(object, &collide_data->hit_result, -1.0f, 0.0f, object->collision->bounce);
    // }
    // else{
    //     correct_velocity(object, &collide_data->hit_result, 1.0f, 0.0f, object->collision->bounce);
    // }

    vector3Sub(object->position, start_pos, &move_amount);
    vector3Add(&move_amount, &object->bounding_box.min, &object->bounding_box.min);
    vector3Add(&move_amount, &object->bounding_box.max, &object->bounding_box.max);

    collide_add_contact(object, &collide_data->hit_result);
}

bool collide_object_to_mesh_swept(struct physics_object* object, struct mesh_collider* mesh, struct Vector3* prev_pos){
    if (object->is_trigger) {
        return false;
    }

    struct object_mesh_collide_data collide_data;
    object_mesh_collide_data_init(&collide_data, prev_pos, mesh, object);

    struct Vector3 start_pos = *object->position;

    struct Vector3 offset;


    vector3Sub(
        object->position,
        prev_pos,
        &offset);

    struct Vector3 box_extent;
    vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &box_extent);
    vector3Scale(&box_extent, &box_extent, 0.5f);
    struct AABB prev_box;
    vector3Sub(prev_pos, &box_extent, &prev_box.min);
    vector3Add(prev_pos, &box_extent, &prev_box.max);

    // span a box from the previous position to the current position to catch all possible triangle collisions
    struct AABB expanded_box = AABBUnion(&prev_box, &object->bounding_box);


    int result_count = 0;
    int aabbCheck_count = 0;
    int max_results = 20;
    NodeProxy results[max_results];

    AABBTree_queryBounds(&mesh->aabbtree, &expanded_box, results, &result_count, &aabbCheck_count, max_results);
    
    bool did_hit = false;
    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABBTreeNode_getData(&mesh->aabbtree, results[j]);

        did_hit = did_hit | collide_object_swept_to_triangle(&collide_data, triangle_index);
    }
    if (!did_hit)
    {
        return false;
    }

    collide_object_swept_bounce(object, &collide_data, &start_pos);

    return true;
}