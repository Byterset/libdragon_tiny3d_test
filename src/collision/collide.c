#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include "../math/matrix.h"
#include "../math/mathf.h"
#include "../time/time.h"
#include <stdio.h>
#include <math.h>


void correct_velocity(struct physics_object* a, struct physics_object* b, struct EpaResult* result, float friction, float bounce) {
    // EPA result normal points toward A (from B to A)
    // Parameters match EPA order:
    // - For terrain: EPA (terrain, object), call (NULL, object) - normal points toward NULL (terrain)
    // - For objects: EPA (a, b), call (a, b) - normal points toward A
    // When A is on top of B, normal points upward (from B toward A)

    // Normal points from B toward A
    Vector3 normal = result->normal;

    // Check if movement is fully constrained for each object
    bool aMovementConstrained = a && (a->is_kinematic || (a->constrain_movement_x && a->constrain_movement_y && a->constrain_movement_z));
    bool bMovementConstrained = b && (b->is_kinematic || (b->constrain_movement_x && b->constrain_movement_y && b->constrain_movement_z));

    // For objects with movement constraints, check if constrained along collision normal
    float invMassA = 0.0f;
    float invMassB = 0.0f;

    if (a && !aMovementConstrained) {
        // Check if movement is constrained along the normal direction
        bool constrainedAlongNormal = (a->constrain_movement_x && fabsf(normal.x) > 0.01f) ||
                                      (a->constrain_movement_y && fabsf(normal.y) > 0.01f) ||
                                      (a->constrain_movement_z && fabsf(normal.z) > 0.01f);
        invMassA = constrainedAlongNormal ? 0.0f : a->_inv_mass;
    }

    if (b && !bMovementConstrained) {
        // Check if movement is constrained along the normal direction (opposite direction)
        bool constrainedAlongNormal = (b->constrain_movement_x && fabsf(normal.x) > 0.01f) ||
                                      (b->constrain_movement_y && fabsf(normal.y) > 0.01f) ||
                                      (b->constrain_movement_z && fabsf(normal.z) > 0.01f);
        invMassB = constrainedAlongNormal ? 0.0f : b->_inv_mass;
    }

    float invMassSum = invMassA + invMassB;

    if (invMassSum == 0.0f) {
        return; // Both objects have constrained movement along collision normal
    }

    // Calculate contact velocity for object A
    Vector3 contactVelA = gZeroVec;
    Vector3 rA = gZeroVec;
    bool hasA = (a != NULL);
    bool hasLinearA = (hasA && invMassA > 0.0f);
    bool hasRotationA = false;

    if (hasA) {
        Vector3 centerOfMassA;
        if (a->rotation) {
            Vector3 rotatedOffset;
            quatMultVector(a->rotation, &a->center_offset, &rotatedOffset);
            vector3Add(a->position, &rotatedOffset, &centerOfMassA);
        } else {
            vector3Add(a->position, &a->center_offset, &centerOfMassA);
        }

        vector3Sub(&result->contactB, &centerOfMassA, &rA);

        contactVelA = a->velocity;
        hasRotationA = a->rotation && !(a->constrain_rotation_x && a->constrain_rotation_y && a->constrain_rotation_z);
        if (hasRotationA) {
            Vector3 angularContribution;
            vector3Cross(&a->angular_velocity, &rA, &angularContribution);
            vector3Add(&contactVelA, &angularContribution, &contactVelA);
        }
    }

    // Calculate contact velocity for object B
    Vector3 contactVelB = gZeroVec;
    Vector3 rB = gZeroVec;
    bool hasB = (b != NULL);
    bool hasLinearB = (hasB && invMassB > 0.0f);
    bool hasRotationB = false;

    if (hasB) {
        Vector3 centerOfMassB;
        if (b->rotation) {
            Vector3 rotatedOffset;
            quatMultVector(b->rotation, &b->center_offset, &rotatedOffset);
            vector3Add(b->position, &rotatedOffset, &centerOfMassB);
        } else {
            vector3Add(b->position, &b->center_offset, &centerOfMassB);
        }

        vector3Sub(&result->contactA, &centerOfMassB, &rB);

        contactVelB = b->velocity;
        hasRotationB = b->rotation && !(b->constrain_rotation_x && b->constrain_rotation_y && b->constrain_rotation_z);
        if (hasRotationB) {
            Vector3 angularContribution;
            vector3Cross(&b->angular_velocity, &rB, &angularContribution);
            vector3Add(&contactVelB, &angularContribution, &contactVelB);
        }
    }

    // Calculate relative velocity (A relative to B)
    Vector3 relVel;
    vector3Sub(&contactVelA, &contactVelB, &relVel);

    // Project relative velocity onto collision normal (which points from B toward A)
    // If vRel > 0: objects are separating (A moving away from B along normal)
    // If vRel < 0: objects are approaching (A moving toward B, collision active)
    float vRel = vector3Dot(&relVel, &normal);

    // If objects are separating, don't apply impulse
    if (vRel >= 0.0f) {
        return;
    }

    // Compute effective bounce with damping for low-velocity contacts
    float effectiveBounce = bounce;
    const float bounceThreshold = 0.5f;
    if (fabsf(vRel) < bounceThreshold) {
        // Smoothly reduce bounce to zero as velocity approaches zero
        // This prevents micro-bouncing in stacked objects
        effectiveBounce = bounce * (fabsf(vRel) / bounceThreshold);
    }

    // Calculate impulse denominator: 1/m_a + 1/m_b + inertia terms
    float denominator = invMassSum;

    // Add rotational inertia term for A
    if (hasA && hasRotationA) {
        Vector3 rCrossN;
        vector3Cross(&rA, &normal, &rCrossN);

        Vector3 torquePerImpulse;
        torquePerImpulse.x = rCrossN.x * a->_inv_local_intertia_tensor.x;
        torquePerImpulse.y = rCrossN.y * a->_inv_local_intertia_tensor.y;
        torquePerImpulse.z = rCrossN.z * a->_inv_local_intertia_tensor.z;

        if (a->constrain_rotation_x) torquePerImpulse.x = 0.0f;
        if (a->constrain_rotation_y) torquePerImpulse.y = 0.0f;
        if (a->constrain_rotation_z) torquePerImpulse.z = 0.0f;

        denominator += vector3Dot(&rCrossN, &torquePerImpulse);
    }

    // Add rotational inertia term for B
    if (hasB && hasRotationB) {
        Vector3 rCrossN;
        vector3Cross(&rB, &normal, &rCrossN);

        Vector3 torquePerImpulse;
        torquePerImpulse.x = rCrossN.x * b->_inv_local_intertia_tensor.x;
        torquePerImpulse.y = rCrossN.y * b->_inv_local_intertia_tensor.y;
        torquePerImpulse.z = rCrossN.z * b->_inv_local_intertia_tensor.z;

        if (b->constrain_rotation_x) torquePerImpulse.x = 0.0f;
        if (b->constrain_rotation_y) torquePerImpulse.y = 0.0f;
        if (b->constrain_rotation_z) torquePerImpulse.z = 0.0f;

        denominator += vector3Dot(&rCrossN, &torquePerImpulse);
    }

    // Add Baumgarte stabilization for penetration correction
    // Old version applied this TWICE (once per object), so we need a stronger factor
    // Also, old version had smaller denominator (single object mass), new has both masses
    float baumgarteBias = 0.0f;
    if (result->penetration > 0.005f) {
        const float baumgarteSlop = 0.02f;  // Allow small penetration
        const float baumgarteFactor = 0.5f;  // Stronger to compensate for single application
        float penetrationError = maxf(result->penetration - baumgarteSlop, 0.0f);
        baumgarteBias = (baumgarteFactor / FIXED_DELTATIME) * penetrationError;
    }

    // Calculate normal impulse magnitude
    // Note: baumgarteBias is NOT divided by denominator (dimensionally inconsistent but works)
    float jN = -(1.0f + effectiveBounce) * vRel / denominator + baumgarteBias;

    if (jN < 0.0f) {
        return;
    }

    // Apply normal impulse to A (in direction of normal)
    // Linear response only if object has non-zero effective mass
    if (hasLinearA) {
        Vector3 impulseA;
        vector3Scale(&normal, &impulseA, jN * invMassA);

        // Apply to linear velocity with per-axis constraints
        Vector3 linearImpulse = impulseA;
        if (a->constrain_movement_x) linearImpulse.x = 0.0f;
        if (a->constrain_movement_y) linearImpulse.y = 0.0f;
        if (a->constrain_movement_z) linearImpulse.z = 0.0f;
        vector3Add(&a->velocity, &linearImpulse, &a->velocity);

        // Update contact velocity with linear impulse
        vector3Add(&contactVelA, &linearImpulse, &contactVelA);
    }

    // Angular response independent of linear constraints
    if (hasRotationA) {
        Vector3 rCrossN;
        vector3Cross(&rA, &normal, &rCrossN);
        vector3Scale(&rCrossN, &rCrossN, jN);
        physics_object_apply_angular_impulse(a, &rCrossN);

        // Update contact velocity with angular contribution
        Vector3 deltaAngularVel;
        deltaAngularVel.x = rCrossN.x * a->_inv_local_intertia_tensor.x;
        deltaAngularVel.y = rCrossN.y * a->_inv_local_intertia_tensor.y;
        deltaAngularVel.z = rCrossN.z * a->_inv_local_intertia_tensor.z;

        if (a->constrain_rotation_x) deltaAngularVel.x = 0.0f;
        if (a->constrain_rotation_y) deltaAngularVel.y = 0.0f;
        if (a->constrain_rotation_z) deltaAngularVel.z = 0.0f;

        Vector3 deltaContactVel;
        vector3Cross(&deltaAngularVel, &rA, &deltaContactVel);
        vector3Add(&contactVelA, &deltaContactVel, &contactVelA);
    }

    // Apply normal impulse to B (opposite direction)
    // Linear response only if object has non-zero effective mass
    if (hasLinearB) {
        Vector3 impulseB;
        vector3Scale(&normal, &impulseB, -jN * invMassB);

        // Apply to linear velocity with per-axis constraints
        Vector3 linearImpulse = impulseB;
        if (b->constrain_movement_x) linearImpulse.x = 0.0f;
        if (b->constrain_movement_y) linearImpulse.y = 0.0f;
        if (b->constrain_movement_z) linearImpulse.z = 0.0f;
        vector3Add(&b->velocity, &linearImpulse, &b->velocity);

        // Update contact velocity with linear impulse
        vector3Add(&contactVelB, &linearImpulse, &contactVelB);
    }

    // Angular response independent of linear constraints
    if (hasRotationB) {
        Vector3 rCrossN;
        vector3Cross(&rB, &normal, &rCrossN);
        vector3Scale(&rCrossN, &rCrossN, -jN);
        physics_object_apply_angular_impulse(b, &rCrossN);

        // Update contact velocity with angular contribution
        Vector3 deltaAngularVel;
        deltaAngularVel.x = rCrossN.x * b->_inv_local_intertia_tensor.x;
        deltaAngularVel.y = rCrossN.y * b->_inv_local_intertia_tensor.y;
        deltaAngularVel.z = rCrossN.z * b->_inv_local_intertia_tensor.z;

        if (b->constrain_rotation_x) deltaAngularVel.x = 0.0f;
        if (b->constrain_rotation_y) deltaAngularVel.y = 0.0f;
        if (b->constrain_rotation_z) deltaAngularVel.z = 0.0f;

        Vector3 deltaContactVel;
        vector3Cross(&deltaAngularVel, &rB, &deltaContactVel);
        vector3Add(&contactVelB, &deltaContactVel, &contactVelB);
    }

    // Handle friction
    if (friction > 0.0f) {
        // Recalculate relative velocity after normal impulse
        vector3Sub(&contactVelA, &contactVelB, &relVel);

        // Get tangent velocity (perpendicular to normal)
        float vN = vector3Dot(&relVel, &normal);
        Vector3 normalPart;
        vector3Scale(&normal, &normalPart, vN);
        Vector3 tangentVelocity;
        vector3Sub(&relVel, &normalPart, &tangentVelocity);

        float tangentSpeed = sqrtf(vector3MagSqrd(&tangentVelocity));
        if (tangentSpeed > 0.0001f) {
            Vector3 tangentDir;
            vector3Scale(&tangentVelocity, &tangentDir, -1.0f / tangentSpeed);

            // Calculate friction denominator
            float frictionDenominator = invMassSum;

            // Add rotational inertia term for A
            if (hasA && hasRotationA) {
                Vector3 rCrossT;
                vector3Cross(&rA, &tangentDir, &rCrossT);

                Vector3 torquePerImpulse;
                torquePerImpulse.x = rCrossT.x * a->_inv_local_intertia_tensor.x;
                torquePerImpulse.y = rCrossT.y * a->_inv_local_intertia_tensor.y;
                torquePerImpulse.z = rCrossT.z * a->_inv_local_intertia_tensor.z;

                if (a->constrain_rotation_x) torquePerImpulse.x = 0.0f;
                if (a->constrain_rotation_y) torquePerImpulse.y = 0.0f;
                if (a->constrain_rotation_z) torquePerImpulse.z = 0.0f;

                frictionDenominator += vector3Dot(&rCrossT, &torquePerImpulse);
            }

            // Add rotational inertia term for B
            if (hasB && hasRotationB) {
                Vector3 rCrossT;
                vector3Cross(&rB, &tangentDir, &rCrossT);

                Vector3 torquePerImpulse;
                torquePerImpulse.x = rCrossT.x * b->_inv_local_intertia_tensor.x;
                torquePerImpulse.y = rCrossT.y * b->_inv_local_intertia_tensor.y;
                torquePerImpulse.z = rCrossT.z * b->_inv_local_intertia_tensor.z;

                if (b->constrain_rotation_x) torquePerImpulse.x = 0.0f;
                if (b->constrain_rotation_y) torquePerImpulse.y = 0.0f;
                if (b->constrain_rotation_z) torquePerImpulse.z = 0.0f;

                frictionDenominator += vector3Dot(&rCrossT, &torquePerImpulse);
            }

            // Coulomb friction
            float jT = tangentSpeed / frictionDenominator;
            float maxFriction = friction * jN;
            if (jT > maxFriction) {
                jT = maxFriction;
            }

            // Apply friction impulse to A
            if (hasLinearA) {
                Vector3 frictionImpulse;
                vector3Scale(&tangentDir, &frictionImpulse, jT * invMassA);

                // Apply to linear velocity with per-axis constraints
                Vector3 linearFrictionImpulse = frictionImpulse;
                if (a->constrain_movement_x) linearFrictionImpulse.x = 0.0f;
                if (a->constrain_movement_y) linearFrictionImpulse.y = 0.0f;
                if (a->constrain_movement_z) linearFrictionImpulse.z = 0.0f;
                vector3Add(&a->velocity, &linearFrictionImpulse, &a->velocity);
            }

            // Apply friction torque independent of linear constraints
            if (hasRotationA) {
                Vector3 rCrossT;
                vector3Cross(&rA, &tangentDir, &rCrossT);
                vector3Scale(&rCrossT, &rCrossT, jT);
                physics_object_apply_angular_impulse(a, &rCrossT);
            }

            // Apply friction impulse to B (opposite direction)
            if (hasLinearB) {
                Vector3 frictionImpulse;
                vector3Scale(&tangentDir, &frictionImpulse, -jT * invMassB);

                // Apply to linear velocity with per-axis constraints
                Vector3 linearFrictionImpulse = frictionImpulse;
                if (b->constrain_movement_x) linearFrictionImpulse.x = 0.0f;
                if (b->constrain_movement_y) linearFrictionImpulse.y = 0.0f;
                if (b->constrain_movement_z) linearFrictionImpulse.z = 0.0f;
                vector3Add(&b->velocity, &linearFrictionImpulse, &b->velocity);
            }

            // Apply friction torque independent of linear constraints
            if (hasRotationB) {
                Vector3 rCrossT;
                vector3Cross(&rB, &tangentDir, &rCrossT);
                vector3Scale(&rCrossT, &rCrossT, -jT);
                physics_object_apply_angular_impulse(b, &rCrossT);
            }
        }
    }
}

void correct_overlap(struct physics_object* a, struct physics_object* b, struct EpaResult* result) {

    const float percent = 1.0f; // penetration correction strength (1.0 = full correction)
    const float slop = 0.00f;  // minimal slop

    float d = fabsf(result->penetration);
    // if (d < slop) return; // ignore negligible overlap

    // Check if movement is fully constrained for each object
    bool aMovementConstrained = a && (a->is_kinematic || (a->constrain_movement_x && a->constrain_movement_y && a->constrain_movement_z));
    bool bMovementConstrained = b && (b->is_kinematic || (b->constrain_movement_x && b->constrain_movement_y && b->constrain_movement_z));

    // For objects with movement constraints, check if constrained along collision normal
    float invMassA = 0.0f;
    float invMassB = 0.0f;

    if (a && !aMovementConstrained) {
        // Check if movement is constrained along the normal direction
        bool constrainedAlongNormal = (a->constrain_movement_x && fabsf(result->normal.x) > 0.01f) ||
                                      (a->constrain_movement_y && fabsf(result->normal.y) > 0.01f) ||
                                      (a->constrain_movement_z && fabsf(result->normal.z) > 0.01f);
        invMassA = constrainedAlongNormal ? 0.0f : a->_inv_mass;
    }

    if (b && !bMovementConstrained) {
        // Check if movement is constrained along the normal direction
        bool constrainedAlongNormal = (b->constrain_movement_x && fabsf(result->normal.x) > 0.01f) ||
                                      (b->constrain_movement_y && fabsf(result->normal.y) > 0.01f) ||
                                      (b->constrain_movement_z && fabsf(result->normal.z) > 0.01f);
        invMassB = constrainedAlongNormal ? 0.0f : b->_inv_mass;
    }

    float invMassSum = invMassA + invMassB;
    if (invMassSum == 0.0f) return; // both have constrained movement along collision normal

    float correctionMag = percent * maxf(d - slop, 0.0f) / invMassSum;

    // Debug output for significant penetrations
    if (d > 0.02f && a && b) {
        debugf("OVERLAP: pen=%.4f, A_id=%d mass=%.2f, B_id=%d mass=%.2f, corrMag=%.4f\n",
               d, a->entity_id, a->mass, b->entity_id, b->mass, correctionMag);
    }

    Vector3 correctionVec;
    vector3Scale(&result->normal, &correctionVec, correctionMag);

    // Apply correction proportional to inverse mass
    if (invMassA > 0.0f)
    {
        Vector3 corrA = correctionVec;
        if (a->constrain_movement_x) corrA.x = 0;
        if (a->constrain_movement_y) corrA.y = 0;
        if (a->constrain_movement_z) corrA.z = 0;
        vector3AddScaled(a->position, &corrA, invMassA, a->position);
    }

    if (invMassB > 0.0f)
    {
        Vector3 corrB = correctionVec;
        if (b->constrain_movement_x) corrB.x = 0;
        if (b->constrain_movement_y) corrB.y = 0;
        if (b->constrain_movement_z) corrB.z = 0;
        vector3AddScaled(b->position, &corrB, -invMassB, b->position);
    }

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
        
        if(object->collision->shape_type == COLLISION_SHAPE_CAPSULE){
            debugf("pen: %.2f, norm: (%.2f, %.2f, %.2f), with STATIC\n", result.penetration, result.normal.x, result.normal.y, result.normal.z);
        }
        correct_overlap(NULL, object, &result);
        correct_velocity(NULL, object, &result, object->collision->friction, object->collision->bounce);
        
        
        
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
    struct EpaResult result;

    Vector3 delta;
    float dist_sq;
    float radii_sum;
    bool is_sphere_sphere = a->collision->shape_type == COLLISION_SHAPE_SPHERE && b->collision->shape_type == COLLISION_SHAPE_SPHERE;
    //Sphere-Sphere Optimization
    if(is_sphere_sphere){
        vector3Sub(a->position, b->position, &delta); //vector from B to A
        dist_sq = vector3MagSqrd(&delta);
        radii_sum = a->collision->shape_data.sphere.radius + b->collision->shape_data.sphere.radius;
        float radii_sum_sq = radii_sum * radii_sum;
        if(dist_sq >= radii_sum_sq)
            return;
    }
    else{
        if (!gjkCheckForOverlap(&simplex, a, physics_object_gjk_support_function, b, physics_object_gjk_support_function, &gRight))
        {
            return;
        }
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
    b->_is_sleeping = false;
    b->_sleep_counter = 0;

    if(is_sphere_sphere){

        float dist = sqrtf(dist_sq);
        result.penetration = radii_sum - dist;
        if (dist > EPSILON)
            vector3Normalize(&delta, &result.normal);
        else
            result.normal = gUp;

        vector3AddScaled(a->position, &result.normal, -a->collision->shape_data.sphere.radius, &result.contactA);
        vector3AddScaled(b->position, &result.normal, b->collision->shape_data.sphere.radius, &result.contactB);
    }
    else{
        bool epa_success = epaSolve(
            &simplex,
            a,
            physics_object_gjk_support_function,
            b,
            physics_object_gjk_support_function,
            &result);

        if (!epa_success)
        {
            debugf("EPA FAILED for collision between objects at (%.2f,%.2f,%.2f) and (%.2f,%.2f,%.2f)\n",
                   a->position->x, a->position->y, a->position->z,
                   b->position->x, b->position->y, b->position->z);
            return; // Skip collision resolution if EPA fails
        }
    }

    float friction = a->collision->friction < b->collision->friction ? a->collision->friction : b->collision->friction;
    float bounce = a->collision->bounce > b->collision->bounce ? a->collision->bounce : b->collision->bounce;

    if (a->collision->shape_type == COLLISION_SHAPE_CAPSULE)
    {
        debugf("pen: %.2f, norm: (%.2f, %.2f, %.2f), other: %d\n", result.penetration, result.normal.x, result.normal.y, result.normal.z, b->entity_id);
    }


    correct_overlap(a, b, &result);
    correct_velocity(a, b, &result, friction, bounce);


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