#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include <stdio.h>
#include <math.h>


void correct_velocity(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce) {
    // Calculate the component of velocity along the normal
    float velocityDot = vector3Dot(&object->velocity, &result->normal);

    // Only proceed if the object is moving toward the collision surface (velocityDot < 0)
    if ((velocityDot < 0) == (ratio < 0)) {
        Vector3 normalVelocity;
        Vector3 tangentVelocity;

        // Calculate normal and tangential components of velocity
        vector3Scale(&result->normal, &normalVelocity, velocityDot);
        vector3Sub(&object->velocity, &normalVelocity, &tangentVelocity);

        // Apply bounce: invert the normal velocity component and scale by bounce factor
        vector3Scale(&normalVelocity, &normalVelocity, -bounce);

        // Apply friction: scale the tangent velocity by (1 - friction)
        vector3Scale(&tangentVelocity, &tangentVelocity, 1.0f - friction);

        // Combine normal and tangential components to get the corrected velocity
        vector3Add(&normalVelocity, &tangentVelocity, &object->velocity);
    }
}

void correct_overlap(struct physics_object* object, struct EpaResult* result, float ratio) {
    if (object->is_fixed) {
        return;
    }

    vector3AddScaled(object->position, &result->normal, result->penetration * ratio, object->position);

}

struct object_mesh_collide_data {
    struct mesh_collider* mesh;
    struct physics_object* object;
    struct mesh_triangle triangle;
};

bool collide_object_to_triangle(struct physics_object* object, struct mesh_collider* mesh, int triangle_index){
    struct mesh_triangle triangle;
    triangle.triangle = mesh->triangles[triangle_index];
    triangle.normal = mesh->normals[triangle_index];
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
        correct_overlap(object, &result, -1.0f);
        correct_velocity(object, &result, -1.0f, object->collision->friction, object->collision->bounce);
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
    int max_results = 20;
    NodeProxy results[max_results];

    AABBTree_queryBounds(&mesh->aabbtree, &object->bounding_box, results, &result_count, max_results);
    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABBTreeNode_getData(&mesh->aabbtree, results[j]);

        collide_object_to_triangle(object, mesh, triangle_index);
    }    
}


/// @brief Handles the collision between two physics objects
///
/// The function first checks if the two object are able and supposed to collide.
/// Then it performs a GJK/EPA collision detection and resolution.
/// @param a physics object a
/// @param b physics object b
/// @return returns 1 if an overlap was resolved (NOTE: a 0 return does not mean no collision happened, only that no overlap was resolved)
void collide_object_to_object(struct physics_object* a, struct physics_object* b) {
    // If the Object don't share any collision layers, don't collide
    if (!(a->collision_layers & b->collision_layers)) {
        return;
    }

    // If the objects are in the same collision group, don't collide (eg. player vs player, or enemy vs enemy)
    if (a->collision_group && (a->collision_group == b->collision_group)) {
        return;
    }

    // triggers can't collide with each other
    if (a->is_trigger && b->is_trigger) {
        return;
    }

    struct Simplex simplex;
    if (!gjkCheckForOverlap(&simplex, a, physics_object_gjk_support_function, b, physics_object_gjk_support_function, &gRight)) {
        return;
    }

    if (a->is_trigger || b->is_trigger) {
        struct contact* contact = collision_scene_new_contact();

        if (!contact) {
            return;
        }

        if (a->is_trigger) {
            contact->normal = gZeroVec;
            contact->point = *a->position;
            contact->other_object = a->entity_id;

            contact->next = b->active_contacts;
            b->active_contacts = contact;
        } else {
            contact->normal = gZeroVec;
            contact->point = *b->position;
            contact->other_object = b->entity_id;

            contact->next = a->active_contacts;
            a->active_contacts = contact;
        }

        return;
    }
    b->is_sleeping = 0;
    b->_sleep_counter = 0;

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
    if(a->is_fixed){
        massRatio = 1.0f;
    }
    if(b->is_fixed){
        massRatio = 0.0f;
    }

    correct_overlap(b, &result, -massRatio);
    correct_velocity(b, &result, -massRatio, friction, bounce);
    correct_overlap(a, &result, (1.0f - massRatio));
    correct_velocity(a, &result, (1.0f - massRatio), friction, bounce);

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