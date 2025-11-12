#include "collide.h"

#include "epa.h"

#include "collision_scene.h"
#include "../util/flags.h"
#include "../math/matrix.h"
#include "../math/mathf.h"
#include "../time/time.h"
#include <stdio.h>
#include <math.h>



void correct_velocity(struct physics_object* object, struct physics_object* other, struct EpaResult* result, float ratio, float friction, float bounce) {
    // Fixed objects don't have their velocity corrected (they're kinematic)
    if (object->is_kinematic) {
        return;
    }

    // EPA normal points from A to B
    // For object A (ratio > 0), we need normal pointing from B to A (flip it)
    // For object B (ratio < 0), we need normal pointing from A to B (keep it)
    Vector3 effectiveNormal;
    if (ratio > 0) {
        vector3Negate(&result->normal, &effectiveNormal);
    } else {
        effectiveNormal = result->normal;
    }

    // Calculate the component of velocity along the effective normal
    float velocityDot = vector3Dot(&object->velocity, &effectiveNormal);

    // Only proceed if the object is moving toward the collision surface (velocityDot < 0)
    if (velocityDot < 0) {

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
        bool has_rotation = object->rotation &&
                            !(object->constrain_rotation_x && object->constrain_rotation_y && object->constrain_rotation_z);
        if (has_rotation) {
            Vector3 angularContribution;
            vector3Cross(&object->angular_velocity, &r, &angularContribution);
            vector3Add(&contactVelocity, &angularContribution, &contactVelocity);
        }

        // Get normal component of contact velocity
        float vRel = vector3Dot(&contactVelocity, &effectiveNormal);

        // Reduce bounce for low-velocity impacts (stabilizes resting contacts)
        float effectiveBounce = bounce;
        if (fabsf(vRel) < 0.5f) {
            // Gradually reduce bounce as velocity approaches zero
            effectiveBounce = bounce * (fabsf(vRel) / 0.5f);
        }

        // Calculate the denominator for impulse calculation
        // j = -(1 + e) * vRel / (1/m + (r × n) · (I^-1 (r × n)))
        float denominator = 1.0f / object->mass;

        if (has_rotation) {
            // r × n
            Vector3 rCrossN;
            vector3Cross(&r, &effectiveNormal, &rCrossN);

            // I^-1 (r × n) - apply inverse inertia tensor
            Vector3 torquePerImpulse;
            torquePerImpulse.x = rCrossN.x * object->_inv_local_intertia_tensor.x;
            torquePerImpulse.y = rCrossN.y * object->_inv_local_intertia_tensor.y;
            torquePerImpulse.z = rCrossN.z * object->_inv_local_intertia_tensor.z;

            // Apply per-axis rotation constraints
            if (object->constrain_rotation_x) torquePerImpulse.x = 0.0f;
            if (object->constrain_rotation_y) torquePerImpulse.y = 0.0f;
            if (object->constrain_rotation_z) torquePerImpulse.z = 0.0f;

            // (r × n) · (I^-1 (r × n))
            float angularEffect = vector3Dot(&rCrossN, &torquePerImpulse);
            denominator += angularEffect;
        }

        // Add Baumgarte stabilization to correct penetration
        // This adds a small bias velocity proportional to penetration depth
        float baumgarteBias = 0.0f;
        if (result->penetration > 0.005f) {
            // Baumgarte factor: how aggressively to correct penetration
            const float baumgarteSlop = 0.02f;  // Allow small penetration
            const float baumgarteFactor = 0.15f; // Correction strength

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
        vector3Scale(&effectiveNormal, &normalImpulse, jN * fabsf(ratio) / object->mass);
        vector3Add(&object->velocity, &normalImpulse, &object->velocity);

        // Update contact velocity with linear impulse contribution
        vector3Add(&contactVelocity, &normalImpulse, &contactVelocity);

        // Apply normal impulse to angular velocity
        if (has_rotation) {
            Vector3 rCrossN;
            vector3Cross(&r, &effectiveNormal, &rCrossN);
            vector3Scale(&rCrossN, &rCrossN, jN * fabsf(ratio));
            physics_object_apply_angular_impulse(object, &rCrossN);

            // Update contact velocity with angular impulse contribution
            // Δω = I^-1 * impulse, so Δv_contact = (Δω) × r
            Vector3 deltaAngularVelocity;
            deltaAngularVelocity.x = rCrossN.x * object->_inv_local_intertia_tensor.x;
            deltaAngularVelocity.y = rCrossN.y * object->_inv_local_intertia_tensor.y;
            deltaAngularVelocity.z = rCrossN.z * object->_inv_local_intertia_tensor.z;

            // Apply rotation constraints
            if (object->constrain_rotation_x) deltaAngularVelocity.x = 0.0f;
            if (object->constrain_rotation_y) deltaAngularVelocity.y = 0.0f;
            if (object->constrain_rotation_z) deltaAngularVelocity.z = 0.0f;

            Vector3 deltaContactVelocity;
            vector3Cross(&deltaAngularVelocity, &r, &deltaContactVelocity);
            vector3Add(&contactVelocity, &deltaContactVelocity, &contactVelocity);
        }

        // Now handle friction
        if (friction > 0.0f) {
            // Calculate RELATIVE velocity for friction
            // - For dynamic-dynamic collisions: relative velocity between two moving objects
            // - For kinematic-dynamic collisions: dynamic object relative to kinematic (e.g., ball vs player)
            // - For static collisions (other == NULL): use absolute velocity (e.g., ball vs ground mesh)
            Vector3 relativeContactVelocity = contactVelocity;

            Vector3 rOther = gZeroVec;
            if (other) {
                // Calculate other object's contact velocity (even if kinematic)
                // We need this for proper relative motion calculation
                Vector3 otherCenterOfMass;
                if (other->rotation) {
                    Vector3 rotatedOffset;
                    quatMultVector(other->rotation, &other->center_offset, &rotatedOffset);
                    vector3Add(other->position, &rotatedOffset, &otherCenterOfMass);
                } else {
                    vector3Add(other->position, &other->center_offset, &otherCenterOfMass);
                }

                vector3Sub(contactPoint, &otherCenterOfMass, &rOther);

                Vector3 otherContactVelocity = other->velocity;
                bool other_has_rotation = other->rotation &&
                                          !(other->constrain_rotation_x && other->constrain_rotation_y && other->constrain_rotation_z);
                if (other_has_rotation) {
                    Vector3 otherAngularContribution;
                    vector3Cross(&other->angular_velocity, &rOther, &otherAngularContribution);
                    vector3Add(&otherContactVelocity, &otherAngularContribution, &otherContactVelocity);
                }

                // Subtract other's velocity to get relative sliding velocity
                vector3Sub(&relativeContactVelocity, &otherContactVelocity, &relativeContactVelocity);
            }

            // Get tangent velocity (perpendicular to normal) using relative velocity
            float vN = vector3Dot(&relativeContactVelocity, &effectiveNormal);
            Vector3 normalPart;
            vector3Scale(&effectiveNormal, &normalPart, vN);
            Vector3 tangentVelocity;
            vector3Sub(&relativeContactVelocity, &normalPart, &tangentVelocity);

            float tangentSpeed = sqrtf(vector3MagSqrd(&tangentVelocity));
            if (tangentSpeed > 0.0001f) {
                // Friction direction (opposite to tangent velocity)
                Vector3 tangentDir;
                vector3Scale(&tangentVelocity, &tangentDir, -1.0f / tangentSpeed);

                // Calculate friction impulse denominator for TWO-BODY friction
                // Proper formula: 1/m_a + 1/m_b + (r_a × t)·(I_a^-1 (r_a × t)) + (r_b × t)·(I_b^-1 (r_b × t))
                float frictionDenominator = 1.0f / object->mass;

                // Add OTHER object's mass contribution (for dynamic-dynamic collisions)
                if (other && !other->is_kinematic) {
                    frictionDenominator += 1.0f / other->mass;
                }

                // Add THIS object's rotational inertia term
                if (has_rotation) {
                    Vector3 rCrossT;
                    vector3Cross(&r, &tangentDir, &rCrossT);

                    // I^-1 (r × t) - apply inverse inertia tensor
                    Vector3 torquePerImpulse;
                    torquePerImpulse.x = rCrossT.x * object->_inv_local_intertia_tensor.x;
                    torquePerImpulse.y = rCrossT.y * object->_inv_local_intertia_tensor.y;
                    torquePerImpulse.z = rCrossT.z * object->_inv_local_intertia_tensor.z;

                    // Apply per-axis rotation constraints
                    if (object->constrain_rotation_x) torquePerImpulse.x = 0.0f;
                    if (object->constrain_rotation_y) torquePerImpulse.y = 0.0f;
                    if (object->constrain_rotation_z) torquePerImpulse.z = 0.0f;

                    float angularEffect = vector3Dot(&rCrossT, &torquePerImpulse);
                    frictionDenominator += angularEffect;
                }

                // Add OTHER object's rotational inertia term (for dynamic-dynamic collisions)
                if (other && !other->is_kinematic && other->rotation) {
                    bool other_has_rotation = !(other->constrain_rotation_x &&
                                                other->constrain_rotation_y &&
                                                other->constrain_rotation_z);
                    if (other_has_rotation) {
                        Vector3 rOtherCrossT;
                        vector3Cross(&rOther, &tangentDir, &rOtherCrossT);

                        Vector3 otherTorquePerImpulse;
                        otherTorquePerImpulse.x = rOtherCrossT.x * other->_inv_local_intertia_tensor.x;
                        otherTorquePerImpulse.y = rOtherCrossT.y * other->_inv_local_intertia_tensor.y;
                        otherTorquePerImpulse.z = rOtherCrossT.z * other->_inv_local_intertia_tensor.z;

                        // Apply constraints
                        if (other->constrain_rotation_x) otherTorquePerImpulse.x = 0.0f;
                        if (other->constrain_rotation_y) otherTorquePerImpulse.y = 0.0f;
                        if (other->constrain_rotation_z) otherTorquePerImpulse.z = 0.0f;

                        float otherAngularEffect = vector3Dot(&rOtherCrossT, &otherTorquePerImpulse);
                        frictionDenominator += otherAngularEffect;
                    }
                    // If other can't rotate, its inertia term is 0 (infinite inertia)
                    // This naturally increases the friction impulse, creating more spin
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
                // Note: Angular friction is applied in the FIRST call only (when ratio < 0)
                // to avoid double-application, and uses FULL jT (not scaled by ratio)
                if (ratio < 0.0f) {
                    if (has_rotation) {
                        Vector3 rCrossT;
                        vector3Cross(&r, &tangentDir, &rCrossT);
                        vector3Scale(&rCrossT, &rCrossT, jT);
                        physics_object_apply_angular_impulse(object, &rCrossT);
                    }

                    // Apply friction impulse to OTHER object's angular velocity (Newton's 3rd law)
                    if (other && !other->is_kinematic && other->rotation) {
                        bool other_has_rotation = !(other->constrain_rotation_x &&
                                                    other->constrain_rotation_y &&
                                                    other->constrain_rotation_z);
                        if (other_has_rotation) {
                            Vector3 rOtherCrossT;
                            vector3Cross(&rOther, &tangentDir, &rOtherCrossT);
                            // Opposite direction (Newton's 3rd law)
                            vector3Scale(&rOtherCrossT, &rOtherCrossT, -jT);
                            physics_object_apply_angular_impulse(other, &rOtherCrossT);
                        }
                    }
                }
            }
        }
    }
}

void correct_overlap(struct physics_object* object, struct EpaResult* result, float ratio) {

    if (object->is_kinematic) {
        return;
    }

    // EPA normal points from A to B
    // For object A (ratio > 0), we need normal pointing from B to A (flip it)
    // For object B (ratio < 0), we need normal pointing from A to B (keep it)
    Vector3 effectiveNormal;
    if (ratio > 0) {
        vector3Negate(&result->normal, &effectiveNormal);
    } else {
        effectiveNormal = result->normal;
    }

    if(object->constrain_movement_x) effectiveNormal.x = 0;
    if(object->constrain_movement_y) effectiveNormal.y = 0;
    if(object->constrain_movement_z) effectiveNormal.z = 0;

    // Apply position correction with slight over-correction for faster convergence
    // This helps resolve stacking penetration more quickly
    const float POSITION_CORRECTION_SCALE = 1.05f; // Can increase to 1.1-1.2 for more aggressive correction
    vector3AddScaled(object->position, &effectiveNormal, -result->penetration * fabsf(ratio) * POSITION_CORRECTION_SCALE, object->position);

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
        correct_velocity(object, NULL, &result, -1.0f, object->collision->friction, object->collision->bounce);
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
        vector3Sub(b->position, a->position, &delta);
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
        float penetration = radii_sum - dist;
        result.penetration = -penetration;
        if (dist > EPSILON)
            vector3Normalize(&delta, &result.normal);
        else
            result.normal = gUp;

        vector3AddScaled(a->position, &result.normal, a->collision->shape_data.sphere.radius, &result.contactA);
        vector3AddScaled(b->position, &result.normal, -b->collision->shape_data.sphere.radius, &result.contactB);
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

    float massRatio = a->mass / (a->mass + b->mass);
    if(a->is_kinematic){
        massRatio = 1.0f;
    }
    if(b->is_kinematic){
        massRatio = 0.0f;
    }

    correct_overlap(b, &result, -massRatio);
    correct_overlap(a, &result, (1.0f - massRatio));
    correct_velocity(b, a, &result, -massRatio, friction, bounce);
    correct_velocity(a, b, &result, (1.0f - massRatio), friction, bounce);

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