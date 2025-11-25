#include "collide_swept.h"

#include <math.h>
#include <float.h>
#include "collision_scene.h"
#include "collide.h"
#include "gjk.h"
#include "epa.h"
#include "raycast.h"
#include "shapes/ray_triangle_intersection.h"
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
    
    // Use raycast for high-speed tunneling prevention
    // This is more robust than EPA on swept hull for thin walls
    Vector3 dir;
    vector3FromTo(collide_data->prev_pos, collide_data->object->position, &dir);
    float dist = vector3Mag(&dir);
    
    if (dist > 0.0001f) {
        raycast ray;
        ray.origin = *collide_data->prev_pos;
        ray.dir = dir;
        vector3Scale(&ray.dir, &ray.dir, 1.0f / dist);
        ray.maxDistance = dist;
        
        struct mesh_triangle triangle;
        triangle.vertices = collide_data->mesh->vertices;
        triangle.triangle = collide_data->mesh->triangles[triangle_index];
        triangle.normal = collide_data->mesh->normals[triangle_index];
        
        raycast_hit hit;
        if (ray_triangle_intersection(&ray, &hit, &triangle)) {
            // We hit the triangle with the center ray
            // Construct a result that looks like EPA result
            collide_data->hit_result.normal = hit.normal;
            collide_data->hit_result.penetration = 0;
            collide_data->hit_result.contactA = hit.point; // On triangle
            collide_data->hit_result.contactB = hit.point; // On object (approx)
            
            // Move object to hit point (minus a small buffer)
            Vector3 back_off;
            vector3Scale(&ray.dir, &back_off, 0.01f); // Back off 1cm
            vector3Sub(&hit.point, &back_off, collide_data->object->position);
            
            return true;
        }
    }

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
        bool ignore = false;
        contact* c = collide_data->object->active_contacts;
        while (c) {
            if (c->other_object == NULL && vector3Dot(&result.normal, &c->constraint->normal) > 0.9f) {
                ignore = true;
                break;
            }
            c = c->next;
        }

        if (!ignore) {
            collide_data->hit_result = result;
            return true;
        }
    }
    *collide_data->object->position = final_pos;

    return false;
}

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
    correct_velocity(object, &collide_data->hit_result, 0.0f, object->collision->bounce);

    vector3Sub(object->position, start_pos, &move_amount);
    vector3Add(&move_amount, &object->bounding_box.min, &object->bounding_box.min);
    vector3Add(&move_amount, &object->bounding_box.max, &object->bounding_box.max);

    //Add new contact to object (object is contact Point B in the case of mesh collision)
    // Cache the contact (entity_a = 0 for static mesh)
    contact_constraint *constraint = cache_contact_constraint(NULL, object, &collide_data->hit_result, 0, object->collision->bounce, false);
    constraint->is_active = false;
    // Still add to old contact list for ground detection logic
    collide_add_contact(object, constraint, NULL);
}


bool collide_object_to_mesh_swept(physics_object* object, struct mesh_collider* mesh, Vector3* prev_pos){
    if (object->is_trigger) {
        return false;
    }

    struct object_mesh_collide_data collide_data;
    object_mesh_collide_data_init(&collide_data, prev_pos, mesh, object);

    Vector3 start_pos = *object->position;

    Vector3 offset;


    vector3Sub(
        object->position,
        prev_pos,
        &offset);

    Vector3 box_extent;
    vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &box_extent);
    vector3Scale(&box_extent, &box_extent, 0.5f);
    AABB prev_box;
    vector3Sub(prev_pos, &box_extent, &prev_box.min);
    vector3Add(prev_pos, &box_extent, &prev_box.max);

    // span a box from the previous position to the current position to catch all possible triangle collisions
    AABB expanded_box = AABBUnion(&prev_box, &object->bounding_box);


    int result_count = 0;
    int max_results = 20;
    node_proxy results[max_results];

    AABB_tree_query_bounds(&mesh->aabbtree, &expanded_box, results, &result_count, max_results);
    
    bool did_hit = false;
    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABB_tree_get_node_data(&mesh->aabbtree, results[j]);

        did_hit = did_hit | collide_object_swept_to_triangle(&collide_data, triangle_index);
    }
    if (!did_hit)
    {
        return false;
    }

    collide_object_swept_bounce(object, &collide_data, &start_pos);

    return true;
}