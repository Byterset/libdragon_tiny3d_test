#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include "../math/matrix.h"
#include <stdio.h>
#include <math.h>



void correct_velocity(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce) {
    // Fixed objects don't have their velocity corrected
    if (object->is_fixed) {
        return;
    }

    // Calculate the component of velocity along the normal
    float velocityDot = vector3Dot(&object->velocity, &result->normal);

    // Only proceed if the object is moving toward the collision surface (velocityDot < 0)
    if ((velocityDot < 0) == (ratio < 0)) {
        Vector3 normalVelocity;
        Vector3 tangentVelocity;

        // Calculate normal and tangential components of velocity
        vector3Scale(&result->normal, &normalVelocity, velocityDot);
        vector3Sub(&object->velocity, &normalVelocity, &tangentVelocity);

        // Apply torque from collision if object can rotate
        if (!object->is_rotation_fixed && object->rotation) {
            // Get contact point (use contactA if ratio > 0, contactB if ratio < 0)
            Vector3* contactPoint = (ratio > 0) ? &result->contactA : &result->contactB;

            // Calculate center of mass in world space
            Vector3 centerOfMass;
            Vector3 rotatedOffset;
            quatMultVector(object->rotation, &object->center_offset, &rotatedOffset);
            vector3Add(object->position, &rotatedOffset, &centerOfMass);

            // Calculate r = contact point - center of mass
            Vector3 r;
            vector3Sub(contactPoint, &centerOfMass, &r);

            // Calculate velocity at contact point: v_contact = v_linear + ω × r
            Vector3 angularContribution;
            vector3Cross(&object->angular_velocity, &r, &angularContribution);
            Vector3 contactVelocity;
            vector3Add(&object->velocity, &angularContribution, &contactVelocity);

            // Get normal and tangent components of contact velocity
            float contactNormalVel = vector3Dot(&contactVelocity, &result->normal);
            Vector3 contactNormalVelocity, contactTangentVelocity;
            vector3Scale(&result->normal, &contactNormalVelocity, contactNormalVel);
            vector3Sub(&contactVelocity, &contactNormalVelocity, &contactTangentVelocity);

            // Apply normal impulse torque (from bounce)
            float normalImpulseMag = -contactNormalVel * (1.0f + bounce) * object->mass;
            Vector3 normalImpulse;
            vector3Scale(&result->normal, &normalImpulse, normalImpulseMag);
            Vector3 normalTorque;
            vector3Cross(&r, &normalImpulse, &normalTorque);
            vector3Scale(&normalTorque, &normalTorque, 3); // TODO: calculate all this correctly with inertia tensor
            physics_object_apply_torque(object, &normalTorque);

            // Apply tangential friction torque
            float tangentSpeed = sqrtf(vector3MagSqrd(&contactTangentVelocity));
            if (tangentSpeed > 0.0001f) {
                // Friction force opposes tangential motion
                Vector3 frictionDirection;
                vector3Scale(&contactTangentVelocity, &frictionDirection, -1.0f / tangentSpeed);

                // Friction impulse magnitude (Coulomb friction model)
                float frictionImpulseMag = friction * fabsf(normalImpulseMag);
                Vector3 frictionImpulse;
                vector3Scale(&frictionDirection, &frictionImpulse, frictionImpulseMag);

                // Apply friction torque
                Vector3 frictionTorque;
                vector3Cross(&r, &frictionImpulse, &frictionTorque);
                vector3Scale(&frictionTorque, &frictionTorque, 3); // TODO
                physics_object_apply_torque(object, &frictionTorque);
            }
        }

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
void collide_object_to_object(struct physics_object* a, struct physics_object* b) {
    // If the Objects don't share any collision layers, don't collide
    if (!(a->collision_layers & b->collision_layers)) {
        return;
    }

    // If the objects are in the same collision group, don't collide (eg. multiple projectiles emmited by enemies)
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
    b->is_sleeping = false;
    b->_sleep_counter = 0;

    struct EpaResult result;

    bool epa_success = epaSolve(
        &simplex,
        a,
        physics_object_gjk_support_function,
        b,
        physics_object_gjk_support_function,
        &result);

    if (!epa_success) {
        debugf("EPA FAILED for collision between objects at (%.2f,%.2f,%.2f) and (%.2f,%.2f,%.2f)\n",
               a->position->x, a->position->y, a->position->z,
               b->position->x, b->position->y, b->position->z);
        return; // Skip collision resolution if EPA fails
    }

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