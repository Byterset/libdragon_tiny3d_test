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

    float invMassA = (a && !a->is_kinematic) ? a->_inv_mass : 0.0f;
    float invMassB = (b && !b->is_kinematic) ? b->_inv_mass : 0.0f;
    float invMassSum = invMassA + invMassB;

    if (invMassSum == 0.0f) {
        return; // Both kinematic/static
    }

    // Normal points from B toward A
    Vector3 normal = result->normal;

    // Calculate contact velocity for object A
    Vector3 contactVelA = gZeroVec;
    Vector3 rA = gZeroVec;
    bool hasA = (a != NULL && invMassA > 0.0f);
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

        vector3Sub(&result->contactA, &centerOfMassA, &rA);

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
    bool hasB = (b != NULL && invMassB > 0.0f);
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

        vector3Sub(&result->contactB, &centerOfMassB, &rB);

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
    const float bounceThreshold = 2.0f;
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
    // This is critical for stacks where position correction alone can't keep up with gravity
    float baumgarteBias = 0.0f;
    if (result->penetration > 0.005f) {
        const float baumgarteSlop = 0.01f;  // Minimal slop
        const float baumgarteFactor = 0.3f;   // Strong correction for stability
        float penetrationError = maxf(result->penetration - baumgarteSlop, 0.0f);
        baumgarteBias = (baumgarteFactor / FIXED_DELTATIME) * penetrationError;
    }

    // Calculate normal impulse magnitude
    // Impulse = (bounce * closing velocity + penetration bias) * effective mass
    float jN = -(1.0f + effectiveBounce) * vRel / denominator + baumgarteBias / denominator;

    if (jN < 0.0f) {
        return;
    }

    // Apply normal impulse to A (in direction of normal)
    if (hasA) {
        Vector3 impulseA;
        vector3Scale(&normal, &impulseA, jN * invMassA);
        vector3Add(&a->velocity, &impulseA, &a->velocity);

        if (hasRotationA) {
            Vector3 rCrossN;
            vector3Cross(&rA, &normal, &rCrossN);
            vector3Scale(&rCrossN, &rCrossN, jN);
            physics_object_apply_angular_impulse(a, &rCrossN);
        }

        // Update A's contact velocity after normal impulse
        vector3Add(&contactVelA, &impulseA, &contactVelA);
        if (hasRotationA) {
            Vector3 deltaAngularVel;
            Vector3 rCrossN;
            vector3Cross(&rA, &normal, &rCrossN);
            deltaAngularVel.x = rCrossN.x * jN * a->_inv_local_intertia_tensor.x;
            deltaAngularVel.y = rCrossN.y * jN * a->_inv_local_intertia_tensor.y;
            deltaAngularVel.z = rCrossN.z * jN * a->_inv_local_intertia_tensor.z;

            if (a->constrain_rotation_x) deltaAngularVel.x = 0.0f;
            if (a->constrain_rotation_y) deltaAngularVel.y = 0.0f;
            if (a->constrain_rotation_z) deltaAngularVel.z = 0.0f;

            Vector3 deltaContactVel;
            vector3Cross(&deltaAngularVel, &rA, &deltaContactVel);
            vector3Add(&contactVelA, &deltaContactVel, &contactVelA);
        }
    }

    // Apply normal impulse to B (opposite direction)
    if (hasB) {
        Vector3 impulseB;
        vector3Scale(&normal, &impulseB, -jN * invMassB);
        vector3Add(&b->velocity, &impulseB, &b->velocity);

        if (hasRotationB) {
            Vector3 rCrossN;
            vector3Cross(&rB, &normal, &rCrossN);
            vector3Scale(&rCrossN, &rCrossN, -jN);
            physics_object_apply_angular_impulse(b, &rCrossN);
        }

        // Update B's contact velocity after normal impulse
        vector3Add(&contactVelB, &impulseB, &contactVelB);
        if (hasRotationB) {
            Vector3 deltaAngularVel;
            Vector3 rCrossN;
            vector3Cross(&rB, &normal, &rCrossN);
            deltaAngularVel.x = rCrossN.x * -jN * b->_inv_local_intertia_tensor.x;
            deltaAngularVel.y = rCrossN.y * -jN * b->_inv_local_intertia_tensor.y;
            deltaAngularVel.z = rCrossN.z * -jN * b->_inv_local_intertia_tensor.z;

            if (b->constrain_rotation_x) deltaAngularVel.x = 0.0f;
            if (b->constrain_rotation_y) deltaAngularVel.y = 0.0f;
            if (b->constrain_rotation_z) deltaAngularVel.z = 0.0f;

            Vector3 deltaContactVel;
            vector3Cross(&deltaAngularVel, &rB, &deltaContactVel);
            vector3Add(&contactVelB, &deltaContactVel, &contactVelB);
        }
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
            if (hasA) {
                Vector3 frictionImpulse;
                vector3Scale(&tangentDir, &frictionImpulse, jT * invMassA);
                vector3Add(&a->velocity, &frictionImpulse, &a->velocity);

                if (hasRotationA) {
                    Vector3 rCrossT;
                    vector3Cross(&rA, &tangentDir, &rCrossT);
                    vector3Scale(&rCrossT, &rCrossT, jT);
                    physics_object_apply_angular_impulse(a, &rCrossT);
                }
            }

            // Apply friction impulse to B (opposite direction)
            if (hasB) {
                Vector3 frictionImpulse;
                vector3Scale(&tangentDir, &frictionImpulse, -jT * invMassB);
                vector3Add(&b->velocity, &frictionImpulse, &b->velocity);

                if (hasRotationB) {
                    Vector3 rCrossT;
                    vector3Cross(&rB, &tangentDir, &rCrossT);
                    vector3Scale(&rCrossT, &rCrossT, -jT);
                    physics_object_apply_angular_impulse(b, &rCrossT);
                }
            }
        }
    }
}

void correct_overlap(struct physics_object* a, struct physics_object* b, struct EpaResult* result) {

    const float percent = 1.0f; // penetration correction strength (1.0 = full correction)
    const float slop = 0.01f;  // minimal slop

    float d = fabsf(result->penetration);
    if (d < slop) return; // ignore negligible overlap

    float invMassA = (a && !a->is_kinematic) ? a->_inv_mass : 0.0f;
    float invMassB = (b && !b->is_kinematic) ? b->_inv_mass : 0.0f;
    float invMassSum = invMassA + invMassB;
    if (invMassSum == 0.0f) return; // both static or kinematic

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