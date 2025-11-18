#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include "../math/matrix.h"
#include "../math/mathf.h"
#include "../time/time.h"
#include <stdio.h>
#include <math.h>

const float baumgarteFactor = 0.3f;
const float slop = 0.003;


void correct_velocity(physics_object* a, physics_object* b, const struct EpaResult* result, float combined_friction, float combined_bounce) {
    // EPA result normal points toward A (from B to A)
    // Parameters match EPA order:
    // - For terrain: EPA (terrain, object), call (NULL, object) - normal points toward NULL (terrain)
    // - For objects: EPA (a, b), call (a, b) - normal points toward A
    // When A is on top of B, normal points upward (from B toward A)

    if (result->penetration < slop)
        return;

    // Normal points from B toward A
    Vector3 normal = result->normal;

    // Check if movement is fully constrained for each object
    bool aMovementConstrained = a && (a->is_kinematic || ((a->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));
    bool bMovementConstrained = b && (b->is_kinematic || ((b->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));

    // For objects with movement constraints, check if constrained along collision normal
    float invMassA = 0.0f;
    float invMassB = 0.0f;

    if (a && !aMovementConstrained) {
        // Check if movement is constrained along the normal direction
        bool constrainedAlongNormal = ((a->constraints & CONSTRAINTS_FREEZE_POSITION_X) && fabsf(normal.x) > 0.01f) ||
                                      ((a->constraints & CONSTRAINTS_FREEZE_POSITION_Y) && fabsf(normal.y) > 0.01f) ||
                                      ((a->constraints & CONSTRAINTS_FREEZE_POSITION_Z) && fabsf(normal.z) > 0.01f);
        invMassA = constrainedAlongNormal ? 0.0f : a->_inv_mass;
    }

    if (b && !bMovementConstrained) {
        // Check if movement is constrained along the normal direction (opposite direction)
        bool constrainedAlongNormal = ((b->constraints & CONSTRAINTS_FREEZE_POSITION_X) && fabsf(normal.x) > 0.01f) ||
                                      ((b->constraints & CONSTRAINTS_FREEZE_POSITION_Y) && fabsf(normal.y) > 0.01f) ||
                                      ((b->constraints & CONSTRAINTS_FREEZE_POSITION_Z) && fabsf(normal.z) > 0.01f);
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
    Quaternion rotation_inverse_a;

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
        hasRotationA = a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL);
        if (hasRotationA) {
            // Pre-calculate inverse rotation for A (used multiple times below)
            quatConjugate(a->rotation, &rotation_inverse_a);

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
    Quaternion rotation_inverse_b;

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
        hasRotationB = b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL);
        if (hasRotationB) {
            // Pre-calculate inverse rotation for B (used multiple times below)
            quatConjugate(b->rotation, &rotation_inverse_b);

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
    float effectiveBounce = combined_bounce;
    const float bounceThreshold = 0.5f;
    if (fabsf(vRel) < bounceThreshold) {
        effectiveBounce = combined_bounce * (fabsf(vRel) / bounceThreshold);
    }

    // Calculate impulse denominator: 1/m_a + 1/m_b + inertia terms
    float denominator = invMassSum;

    // Add rotational inertia term for A
    if (hasA && hasRotationA) {
        Vector3 rCrossN;
        vector3Cross(&rA, &normal, &rCrossN);

        // Transform torque to local space (using pre-calculated inverse)
        Vector3 local_rCrossN;
        quatMultVector(&rotation_inverse_a, &rCrossN, &local_rCrossN);

        // Apply local inertia tensor
        Vector3 local_torquePerImpulse;
        local_torquePerImpulse.x = local_rCrossN.x * a->_inv_local_intertia_tensor.x;
        local_torquePerImpulse.y = local_rCrossN.y * a->_inv_local_intertia_tensor.y;
        local_torquePerImpulse.z = local_rCrossN.z * a->_inv_local_intertia_tensor.z;

        // Apply constraints in LOCAL space
        if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;

        // Transform back to world space
        Vector3 torquePerImpulse;
        quatMultVector(a->rotation, &local_torquePerImpulse, &torquePerImpulse);

        denominator += vector3Dot(&rCrossN, &torquePerImpulse);
    }

    // Add rotational inertia term for B
    if (hasB && hasRotationB) {
        Vector3 rCrossN;
        vector3Cross(&rB, &normal, &rCrossN);

        // Transform torque to local space (using pre-calculated inverse)
        Vector3 local_rCrossN;
        quatMultVector(&rotation_inverse_b, &rCrossN, &local_rCrossN);

        // Apply local inertia tensor
        Vector3 local_torquePerImpulse;
        local_torquePerImpulse.x = local_rCrossN.x * b->_inv_local_intertia_tensor.x;
        local_torquePerImpulse.y = local_rCrossN.y * b->_inv_local_intertia_tensor.y;
        local_torquePerImpulse.z = local_rCrossN.z * b->_inv_local_intertia_tensor.z;

        // Apply constraints in LOCAL space
        if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;

        // Transform back to world space
        Vector3 torquePerImpulse;
        quatMultVector(b->rotation, &local_torquePerImpulse, &torquePerImpulse);

        denominator += vector3Dot(&rCrossN, &torquePerImpulse);
    }

    float baumgarteBias = (baumgarteFactor / FIXED_DELTATIME) * result->penetration;


    if (denominator < EPSILON) denominator = EPSILON;

    // Calculate normal impulse magnitude
    float jN = (-(1.0f + effectiveBounce) * vRel + baumgarteBias) / denominator;

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
        if (a->constraints & CONSTRAINTS_FREEZE_POSITION_X) linearImpulse.x = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Y) linearImpulse.y = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Z) linearImpulse.z = 0.0f;
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
        // Transform angular impulse to local space (using pre-calculated inverse)
        Vector3 local_rCrossN;
        quatMultVector(&rotation_inverse_a, &rCrossN, &local_rCrossN);

        // Apply local inertia tensor
        Vector3 local_deltaAngularVel;
        local_deltaAngularVel.x = local_rCrossN.x * a->_inv_local_intertia_tensor.x;
        local_deltaAngularVel.y = local_rCrossN.y * a->_inv_local_intertia_tensor.y;
        local_deltaAngularVel.z = local_rCrossN.z * a->_inv_local_intertia_tensor.z;

        // Apply constraints in LOCAL space
        if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_deltaAngularVel.x = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_deltaAngularVel.y = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_deltaAngularVel.z = 0.0f;

        // Transform back to world space
        Vector3 deltaAngularVel;
        quatMultVector(a->rotation, &local_deltaAngularVel, &deltaAngularVel);

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
        if (b->constraints & CONSTRAINTS_FREEZE_POSITION_X) linearImpulse.x = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Y) linearImpulse.y = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Z) linearImpulse.z = 0.0f;
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
        // Transform angular impulse to local space (using pre-calculated inverse)
        Vector3 local_rCrossN;
        quatMultVector(&rotation_inverse_b, &rCrossN, &local_rCrossN);

        // Apply local inertia tensor
        Vector3 local_deltaAngularVel;
        local_deltaAngularVel.x = local_rCrossN.x * b->_inv_local_intertia_tensor.x;
        local_deltaAngularVel.y = local_rCrossN.y * b->_inv_local_intertia_tensor.y;
        local_deltaAngularVel.z = local_rCrossN.z * b->_inv_local_intertia_tensor.z;

        // Apply constraints in LOCAL space
        if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_deltaAngularVel.x = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_deltaAngularVel.y = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_deltaAngularVel.z = 0.0f;

        // Transform back to world space
        Vector3 deltaAngularVel;
        quatMultVector(b->rotation, &local_deltaAngularVel, &deltaAngularVel);

        Vector3 deltaContactVel;
        vector3Cross(&deltaAngularVel, &rB, &deltaContactVel);
        vector3Add(&contactVelB, &deltaContactVel, &contactVelB);
    }

    #ifndef DEBUG_IGNORE_FRICTION
    // Handle friction
    if (combined_friction > 0.0f) {
        // Recalculate relative velocity after normal impulse
        vector3Sub(&contactVelA, &contactVelB, &relVel);

        // Get tangent velocity (perpendicular to normal)
        float vN = vector3Dot(&relVel, &normal);
        Vector3 normalPart;
        vector3Scale(&normal, &normalPart, vN);
        Vector3 tangentVelocity;
        vector3Sub(&relVel, &normalPart, &tangentVelocity);

        float tangentSpeed = sqrtf(vector3MagSqrd(&tangentVelocity));
        if (tangentSpeed > 0.002f) {
            Vector3 tangentDir;
            vector3Scale(&tangentVelocity, &tangentDir, -1.0f / tangentSpeed);

            // Calculate friction denominator
            float frictionDenominator = invMassSum;

            // Add rotational inertia term for A
            if (hasA && hasRotationA) {
                Vector3 rCrossT;
                vector3Cross(&rA, &tangentDir, &rCrossT);

                // Transform torque to local space (using pre-calculated inverse)
                Vector3 local_rCrossT;
                quatMultVector(&rotation_inverse_a, &rCrossT, &local_rCrossT);

                // Apply local inertia tensor
                Vector3 local_torquePerImpulse;
                local_torquePerImpulse.x = local_rCrossT.x * a->_inv_local_intertia_tensor.x;
                local_torquePerImpulse.y = local_rCrossT.y * a->_inv_local_intertia_tensor.y;
                local_torquePerImpulse.z = local_rCrossT.z * a->_inv_local_intertia_tensor.z;

                // Apply constraints in LOCAL space
                if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
                if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
                if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;

                // Transform back to world space
                Vector3 torquePerImpulse;
                quatMultVector(a->rotation, &local_torquePerImpulse, &torquePerImpulse);

                frictionDenominator += vector3Dot(&rCrossT, &torquePerImpulse);
            }

            // Add rotational inertia term for B
            if (hasB && hasRotationB) {
                Vector3 rCrossT;
                vector3Cross(&rB, &tangentDir, &rCrossT);

                // Transform torque to local space (using pre-calculated inverse)
                Vector3 local_rCrossT;
                quatMultVector(&rotation_inverse_b, &rCrossT, &local_rCrossT);

                // Apply local inertia tensor
                Vector3 local_torquePerImpulse;
                local_torquePerImpulse.x = local_rCrossT.x * b->_inv_local_intertia_tensor.x;
                local_torquePerImpulse.y = local_rCrossT.y * b->_inv_local_intertia_tensor.y;
                local_torquePerImpulse.z = local_rCrossT.z * b->_inv_local_intertia_tensor.z;

                // Apply constraints in LOCAL space
                if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
                if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
                if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;

                // Transform back to world space
                Vector3 torquePerImpulse;
                quatMultVector(b->rotation, &local_torquePerImpulse, &torquePerImpulse);

                frictionDenominator += vector3Dot(&rCrossT, &torquePerImpulse);
            }

            // Coulomb friction
            float jT = tangentSpeed / frictionDenominator;
            float maxFriction = combined_friction * jN;
            if (jT > maxFriction) {
                jT = maxFriction;
            }

            // Apply friction impulse to A
            if (hasLinearA) {
                Vector3 frictionImpulse;
                vector3Scale(&tangentDir, &frictionImpulse, jT * invMassA);

                // Apply to linear velocity with per-axis constraints
                Vector3 linearFrictionImpulse = frictionImpulse;
                if (a->constraints & CONSTRAINTS_FREEZE_POSITION_X) linearFrictionImpulse.x = 0.0f;
                if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Y) linearFrictionImpulse.y = 0.0f;
                if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Z) linearFrictionImpulse.z = 0.0f;
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
                if (b->constraints & CONSTRAINTS_FREEZE_POSITION_X) linearFrictionImpulse.x = 0.0f;
                if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Y) linearFrictionImpulse.y = 0.0f;
                if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Z) linearFrictionImpulse.z = 0.0f;
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
    #endif
}

void correct_overlap(physics_object* a, physics_object* b, const struct EpaResult* result) {

    const float percent = 0.7f;

    if (result->penetration < slop) return;

    // Check if movement is fully constrained for each object
    bool aMovementConstrained = a && (a->is_kinematic || ((a->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));
    bool bMovementConstrained = b && (b->is_kinematic || ((b->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));

    // For objects with movement constraints, check if constrained along collision normal
    float invMassA = 0.0f;
    float invMassB = 0.0f;

    // Calculate the effective normal after applying constraints
    Vector3 effectiveNormalA = result->normal;
    Vector3 effectiveNormalB = result->normal;;

    float normal_dot_inv = 1.0f / vector3Dot(&result->normal, &result->normal);
    if (a && !aMovementConstrained) {
        if (a->constraints & CONSTRAINTS_FREEZE_POSITION_X)
            effectiveNormalA.x = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)
            effectiveNormalA.y = 0.0f;
        if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)
            effectiveNormalA.z = 0.0f;
        // Project the effective normal onto the actual normal to get the "effective mobility"
        float normalDotA = vector3Dot(&effectiveNormalA, &result->normal);
        invMassA = a->_inv_mass * (normalDotA * normalDotA) * normal_dot_inv;
    }

    if (b && !bMovementConstrained) {
        if (b->constraints & CONSTRAINTS_FREEZE_POSITION_X)
            effectiveNormalB.x = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)
            effectiveNormalB.y = 0.0f;
        if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)
            effectiveNormalB.z = 0.0f;
        // Project the effective normal onto the actual normal to get the "effective mobility"
        float normalDotB = vector3Dot(&effectiveNormalB, &result->normal);
        invMassB = b->_inv_mass * (normalDotB * normalDotB) * normal_dot_inv;
    }

    float invMassSum = invMassA + invMassB;
    if (invMassSum == 0.0f) return;

    float correctionMag = (percent * result->penetration) / invMassSum;

    // Apply correction proportional to inverse mass
    if (a && invMassA > 0.0f)
    {
        vector3AddScaled(a->position, &effectiveNormalA, correctionMag * invMassA, a->position);
    }

    if (b && invMassB > 0.0f)
    {
        vector3AddScaled(b->position, &effectiveNormalB, -correctionMag * invMassB, b->position);
    }

}


bool collide_object_to_triangle(physics_object* object, const struct mesh_collider* mesh, int triangle_index){
    struct mesh_triangle triangle;
    triangle.triangle = mesh->triangles[triangle_index];
    triangle.normal = mesh->normals[triangle_index];
    triangle.vertices = mesh->vertices;

    struct Simplex simplex;
    Vector3 firstDir = gRight;
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, object, physics_object_gjk_support_function, &firstDir))
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
        correct_overlap(NULL, object, &result);
        correct_velocity(NULL, object, &result, object->collision->friction, object->collision->bounce);
        
        //Add new contact to object (object is contact Point B in the case of mesh collision)
        collide_add_contact(object, &result, true, 0);

        return true;
    }

    return false;
}

void collide_object_to_mesh(physics_object* object, const struct mesh_collider* mesh) {   
    if (object->is_trigger) {
        return;
    }

    int result_count = 0;
    int max_results = 20;
    node_proxy results[max_results];

    AABB_tree_query_bounds(&mesh->aabbtree, &object->bounding_box, results, &result_count, max_results);
    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABB_tree_get_node_data(&mesh->aabbtree, results[j]);

        collide_object_to_triangle(object, mesh, triangle_index);
    }    
}


void collide_object_to_object(physics_object* a, physics_object* b) {
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
        vector3FromTo(b->position, a->position, &delta);
        dist_sq = vector3MagSqrd(&delta);
        radii_sum = a->collision->shape_data.sphere.radius + b->collision->shape_data.sphere.radius;
        float radii_sum_sq = radii_sum * radii_sum;
        if(dist_sq >= radii_sum_sq)
            return;
    }
    else{
        Vector3 firstDir = gRight;
        if (!gjkCheckForOverlap(&simplex, a, physics_object_gjk_support_function, b, physics_object_gjk_support_function, &firstDir))
        {
            return;
        }
    }


    if (a->is_trigger || b->is_trigger) {
        contact* contact = collision_scene_new_contact();

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
    if (a && !a->is_kinematic) { a->_is_sleeping = false; a->_sleep_counter = 0; }
    if (b && !b->is_kinematic) { b->_is_sleeping = false; b->_sleep_counter = 0; }


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
            return; // Skip collision resolution if EPA fails
        }
    }

    //Combined friction and Bounce is just minimum of both
    float combined_friction = minf(a->collision->friction, b->collision->friction);
    float combined_bounce = minf(a->collision->bounce, b->collision->bounce);


    correct_overlap(a, b, &result);
    correct_velocity(a, b, &result, combined_friction, combined_bounce);

    collide_add_contact(a, &result, false, b ? b->entity_id : 0);
    collide_add_contact(b, &result, true, a ? a->entity_id : 0);
}

void collide_add_contact(physics_object* object, const struct EpaResult* result, bool is_B, entity_id other_id) {
    contact* contact = collision_scene_new_contact();

    if (!contact) {
        return;
    }

    if(is_B){
        vector3Negate(&result->normal, &contact->normal);
        contact->point = result->contactB;
    }
    else{
        contact->normal = result->normal;
        contact->point = result->contactA;
    }

    contact->other_object = other_id;

    contact->next = object->active_contacts;
    object->active_contacts = contact;
}