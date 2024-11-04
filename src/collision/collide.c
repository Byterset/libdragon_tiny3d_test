#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include <stdio.h>
#include <math.h>


void correct_velocity(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce) {
    float velocityDot = vector3Dot(&object->velocity, &result->normal);

    if ((velocityDot < 0) == (ratio < 0)) {
        struct Vector3 tangentVelocity;

        vector3AddScaled(&object->velocity, &result->normal, -velocityDot, &tangentVelocity);
        vector3Scale(&tangentVelocity, &tangentVelocity, 1.0f - friction);

        vector3AddScaled(&tangentVelocity, &result->normal, velocityDot * -bounce, &tangentVelocity);
        physics_object_set_velocity(object, &tangentVelocity);
    }
}

void correct_overlap(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce) {
    if (object->is_fixed) {
        return;
    }
    struct Vector3 correction;
    vector3Scale(&result->normal, &correction, result->penetration * ratio);
    physics_object_translate_no_force(object, &correction);
    
    float angle = acosf(vector3Dot(&gUp, &result->normal)); // gives the angle between the normal and the up vector in radians
    struct collision_scene* scene = collision_scene_get();
    physics_object_recalculate_aabb(object);
    AABBTree_moveNode(&scene->object_aabbtree, object->aabb_tree_node, object->bounding_box, &correction);
    
    if (correction.y > 0.0f && result->normal.y > 0.5f) {
        struct Vector3 vel = physics_object_get_velocity(object);
        if(vel.y < 0.0f){
            vel.y = 0.0f;
            physics_object_set_velocity(object, &vel);
        }
        
    }
    // correct_velocity(object, result, ratio, friction, bounce);

}

struct object_mesh_collide_data {
    struct mesh_collider* mesh;
    struct physics_object* object;
    struct mesh_triangle triangle;
};

bool collide_object_to_triangle(struct physics_object* object, struct mesh_collider* mesh, int triangle_index){
    struct mesh_triangle triangle;
    triangle.triangle = mesh->triangles[triangle_index];
    triangle.vertices = mesh->vertices;

    struct Simplex simplex;
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, object, physics_object_gjk_support_function, &gRight))
    {
        return false;
    }
    struct EpaResult result;
    if (epaSolve(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            object,
            physics_object_gjk_support_function,
            &result))
    {
        correct_overlap(object, &result, -1.0f, object->collision->friction, object->collision->bounce);
        collide_add_contact(object, &result);
        return true;
    }

    return false;
}

void collide_object_to_mesh(struct physics_object* object, struct mesh_collider* mesh) {   
    if (object->is_trigger) {
        return;
    }

    int result_count = 0;
    int aabbCheck_count = 0;
    int max_results = 20;
    NodeProxy results[max_results];

    AABBTree_queryBounds(&mesh->aabbtree, &object->bounding_box, results, &result_count, &aabbCheck_count, max_results);
    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABBTreeNode_getData(&mesh->aabbtree, results[j]);

        collide_object_to_triangle(object, mesh, triangle_index);
    }    
}

void collide_object_to_object(struct physics_object* a, struct physics_object* b) {
    if (!(a->collision_layers & b->collision_layers)) {
        return;
    }

    if (a->collision_group && a->collision_group == b->collision_group) {
        return;
    }

    if (a->is_trigger && b->is_trigger) {
        return;
    }

    struct Simplex simplex;
    if (!gjkCheckForOverlap(&simplex, a, physics_object_gjk_support_function, b, physics_object_gjk_support_function, &gForward)) {
        return;
    }

    if (a->is_trigger || b->is_trigger) {
        struct contact* contact = collision_scene_new_contact();

        if (!contact) {
            return;
        }

        if (b->is_trigger) {
            contact->normal = gZeroVec;
            contact->point = *a->position;
            contact->other_object = a ? a->entity_id : 0;

            contact->next = b->active_contacts;
            b->active_contacts = contact;
        } else {
            contact->normal = gZeroVec;
            contact->point = *b->position;
            contact->other_object = b ? b->entity_id : 0;

            contact->next = a->active_contacts;
            a->active_contacts = contact;
        }

        return;
    }

    struct EpaResult result;

    epaSolve(
        &simplex, 
        a, 
        physics_object_gjk_support_function, 
        b, 
        physics_object_gjk_support_function, 
        &result);

    float friction = a->collision->friction < b->collision->friction ? a->collision->friction : b->collision->friction;
    float bounce = a->collision->friction > b->collision->friction ? a->collision->friction : b->collision->friction;

    float massRatio = a->mass / (a->mass + b->mass);

    correct_overlap(b, &result, -massRatio, friction, bounce);
    correct_overlap(a, &result, (1.0f - massRatio), friction, bounce);

    struct contact* contact = collision_scene_new_contact();

    if (!contact) {
        return;
    }

    contact->normal = result.normal;
    contact->point = result.contactA;
    contact->other_object = a ? a->entity_id : 0;

    contact->next = b->active_contacts;
    b->active_contacts = contact;

    contact = collision_scene_new_contact();

    if (!contact) {
        return;
    }
    
    vector3Negate(&result.normal, &contact->normal);
    contact->point = result.contactB;
    contact->other_object = b ? b->entity_id : 0;

    contact->next = a->active_contacts;
    a->active_contacts = contact;
}

void collide_add_contact(struct physics_object* object, struct EpaResult* result) {
    struct contact* contact = collision_scene_new_contact();

    if (!contact) {
        return;
    }

    contact->normal = result->normal;
    contact->point = result->contactA;
    contact->other_object = 0;

    contact->next = object->active_contacts;
    object->active_contacts = contact;
}