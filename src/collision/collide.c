#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include "../math/matrix.h"
#include "../math/mathf.h"
#include "../time/time.h"
#include <stdio.h>
#include <math.h>



void correct_velocity(struct physics_object* object, struct EpaResult* result, float ratio, float friction, float bounce) {
    // Fixed objects don't have their velocity corrected (they're kinematic)
    if (object->is_kinematic) {
        return;
    }

    // Calculate the component of velocity along the normal
    float velocityDot = vector3Dot(&object->velocity, &result->normal);

    // Only proceed if the object is moving toward the collision surface (velocityDot < 0)
    if ((velocityDot < 0) == (ratio < 0)) {

        // Get contact point (use contactA if ratio > 0, contactB if ratio < 0)
        Vector3* contactPoint = (ratio > 0) ? &result->contactA : &result->contactB;

        // Calculate center of mass in world space
        Vector3 centerOfMass;
        if (object->rotation) {
            Vector3 rotatedOffset;
            quatMultVector(object->rotation, &object->center_offset, &rotatedOffset);
            vector3Add(object->position, &rotatedOffset, &centerOfMass);
        } else {
            vector3Add(object->position, &object->center_offset, &centerOfMass);
        }

        // Calculate r = contact point - center of mass
        Vector3 r;
        vector3Sub(contactPoint, &centerOfMass, &r);

        // Calculate velocity at contact point: v_contact = v_linear + ω × r
        Vector3 contactVelocity = object->velocity;
        if (!object->is_rotation_fixed && object->rotation) {
            Vector3 angularContribution;
            vector3Cross(&object->angular_velocity, &r, &angularContribution);
            vector3Add(&contactVelocity, &angularContribution, &contactVelocity);
        }

        // Get normal component of contact velocity
        float vRel = vector3Dot(&contactVelocity, &result->normal);

        // Reduce bounce for low-velocity impacts (stabilizes resting contacts)
        float effectiveBounce = bounce;
        if (fabsf(vRel) < 0.5f) {
            // Gradually reduce bounce as velocity approaches zero
            effectiveBounce = bounce * (fabsf(vRel) / 0.5f);
        }

        // Calculate the denominator for impulse calculation
        // j = -(1 + e) * vRel / (1/m + (r × n) · (I^-1 (r × n)))
        float denominator = 1.0f / object->mass;

        if (!object->is_rotation_fixed && object->rotation) {
            // r × n
            Vector3 rCrossN;
            vector3Cross(&r, &result->normal, &rCrossN);

            // I^-1 (r × n) - apply inverse inertia tensor
            Vector3 torquePerImpulse;
            torquePerImpulse.x = rCrossN.x * object->_inv_local_intertia_tensor.x;
            torquePerImpulse.y = rCrossN.y * object->_inv_local_intertia_tensor.y;
            torquePerImpulse.z = rCrossN.z * object->_inv_local_intertia_tensor.z;

            // (r × n) · (I^-1 (r × n))
            float angularEffect = vector3Dot(&rCrossN, &torquePerImpulse);
            denominator += angularEffect;
        }

        // Add Baumgarte stabilization to correct penetration
        // This adds a small bias velocity proportional to penetration depth
        float baumgarteBias = 0.0f;
        if (result->penetration > 0.005f) {
            // Baumgarte factor: how aggressively to correct penetration
            const float baumgarteSlop = 0.01f;  // Allow small penetration
            const float baumgarteFactor = 0.3f; // Correction strength

            float penetrationError = maxf(result->penetration - baumgarteSlop, 0.0f);
            baumgarteBias = (baumgarteFactor / FIXED_DELTATIME) * penetrationError;
        }

        // Calculate normal impulse magnitude
        float jN = -(1.0f + effectiveBounce) * vRel / denominator + baumgarteBias;

        // Don't apply impulse if objects are separating
        if (jN < 0.0f) {
            return;
        }

        // Apply normal impulse to linear velocity
        Vector3 normalImpulse;
        vector3Scale(&result->normal, &normalImpulse, jN * fabsf(ratio) / object->mass);
        vector3Add(&object->velocity, &normalImpulse, &object->velocity);

        // Apply normal impulse to angular velocity
        if (!object->is_rotation_fixed && object->rotation) {
            Vector3 rCrossN;
            vector3Cross(&r, &result->normal, &rCrossN);
            vector3Scale(&rCrossN, &rCrossN, jN * fabsf(ratio));
            physics_object_apply_angular_impulse(object, &rCrossN);
        }

        // Now handle friction
        if (friction > 0.0f) {
            // Recalculate contact velocity after normal impulse
            contactVelocity = object->velocity;
            if (!object->is_rotation_fixed && object->rotation) {
                Vector3 angularContribution;
                vector3Cross(&object->angular_velocity, &r, &angularContribution);
                vector3Add(&contactVelocity, &angularContribution, &contactVelocity);
            }

            // Get tangent velocity (perpendicular to normal)
            float vN = vector3Dot(&contactVelocity, &result->normal);
            Vector3 normalPart;
            vector3Scale(&result->normal, &normalPart, vN);
            Vector3 tangentVelocity;
            vector3Sub(&contactVelocity, &normalPart, &tangentVelocity);

            float tangentSpeed = sqrtf(vector3MagSqrd(&tangentVelocity));
            if (tangentSpeed > 0.0001f) {
                // Friction direction (opposite to tangent velocity)
                Vector3 tangentDir;
                vector3Scale(&tangentVelocity, &tangentDir, -1.0f / tangentSpeed);

                // Calculate friction impulse denominator
                float frictionDenominator = 1.0f / object->mass;

                if (!object->is_rotation_fixed && object->rotation) {
                    Vector3 rCrossT;
                    vector3Cross(&r, &tangentDir, &rCrossT);

                    Vector3 torquePerImpulse;
                    torquePerImpulse.x = rCrossT.x / object->_local_inertia_tensor.x;
                    torquePerImpulse.y = rCrossT.y / object->_local_inertia_tensor.y;
                    torquePerImpulse.z = rCrossT.z / object->_local_inertia_tensor.z;

                    float angularEffect = vector3Dot(&rCrossT, &torquePerImpulse);
                    frictionDenominator += angularEffect;
                }

                // Coulomb friction: jT = min(μ * jN, tangent impulse needed to stop sliding)
                float jT = tangentSpeed / frictionDenominator;
                float maxFriction = friction * jN;
                if (jT > maxFriction) {
                    jT = maxFriction;
                }

                // Apply friction impulse to linear velocity
                Vector3 frictionImpulse;
                vector3Scale(&tangentDir, &frictionImpulse, jT * fabsf(ratio) / object->mass);
                vector3Add(&object->velocity, &frictionImpulse, &object->velocity);

                // Apply friction impulse to angular velocity
                if (!object->is_rotation_fixed && object->rotation) {
                    Vector3 rCrossT;
                    vector3Cross(&r, &tangentDir, &rCrossT);
                    vector3Scale(&rCrossT, &rCrossT, jT * fabsf(ratio));
                    physics_object_apply_angular_impulse(object, &rCrossT);
                }
            }
        }
    }
}

void correct_overlap(struct physics_object* object, struct EpaResult* result, float ratio) {
    if (object->is_kinematic) {
        return;
    }

    // Apply position correction with slight over-correction for faster convergence
    // This helps resolve stacking penetration more quickly
    const float POSITION_CORRECTION_SCALE = 1.0f; // Can increase to 1.1-1.2 for more aggressive correction
    vector3AddScaled(object->position, &result->normal, result->penetration * ratio * POSITION_CORRECTION_SCALE, object->position);

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
    float bounce = a->collision->bounce > b->collision->bounce ? a->collision->bounce : b->collision->bounce;

    float massRatio = a->mass / (a->mass + b->mass);
    if(a->is_kinematic){
        massRatio = 1.0f;
    }
    if(b->is_kinematic){
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