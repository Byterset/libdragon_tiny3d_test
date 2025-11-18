#include "collision_scene.h"

#include <malloc.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <libdragon.h>
#include "../time/time.h"

#include "mesh_collider.h"
#include "../resource/mesh_collider.h"
#include "collide.h"
#include "collide_swept.h"
#include "../util/hash_map.h"

#include "../render/defs.h"
#include "../game/gamestate.h"

struct collision_scene g_scene;

void collision_scene_reset() {
    free(g_scene.elements);
    free(g_scene.all_contacts);
    free(g_scene.cached_contacts);
    AABB_tree_free(&g_scene.object_aabbtree);
    hash_map_destroy(&g_scene.entity_mapping);

    hash_map_init(&g_scene.entity_mapping, MAX_PHYSICS_OBJECTS);
    AABB_tree_init(&g_scene.object_aabbtree, MAX_PHYSICS_OBJECTS);
    g_scene.elements = malloc(sizeof(struct collision_scene_element) * MAX_PHYSICS_OBJECTS);
    g_scene.capacity = MAX_PHYSICS_OBJECTS;
    g_scene.objectCount = 0;
    if(g_scene.mesh_collider){
        mesh_collider_release(g_scene.mesh_collider);
        g_scene.mesh_collider = NULL;
    }

    g_scene.all_contacts = malloc(sizeof(contact) * MAX_ACTIVE_CONTACTS);
    g_scene.next_free_contact = &g_scene.all_contacts[0];

    for (int i = 0; i + 1 < MAX_ACTIVE_CONTACTS; ++i)
    {
        g_scene.all_contacts[i].next = &g_scene.all_contacts[i + 1];
    }

    g_scene.all_contacts[MAX_ACTIVE_CONTACTS - 1].next = NULL;

    // Initialize constraint cache for iterative solver
    g_scene.cached_contacts = malloc(sizeof(contact_constraint) * MAX_CACHED_CONTACTS);
    g_scene.cached_contact_count = 0;
}

struct collision_scene* collision_scene_get() {
    return &g_scene;
}

void collision_scene_add(physics_object* object) {
    if (g_scene.objectCount >= g_scene.capacity) {
        g_scene.capacity *= 2;
        g_scene.elements = realloc(g_scene.elements, sizeof(struct collision_scene_element) * g_scene.capacity);
    }

    struct collision_scene_element* next = &g_scene.elements[g_scene.objectCount];

    next->object = object;

    g_scene.objectCount += 1;

    hash_map_set(&g_scene.entity_mapping, object->entity_id, object);
    object->_aabb_tree_node_id = AABB_tree_create_node(&g_scene.object_aabbtree, object->bounding_box, object);
}

/// @brief Returns the physics object associated with the given entity id if it exists in the collision scene.
physics_object* collision_scene_find_object(entity_id id) {
    if (!id) {
        return 0;
    }

    return hash_map_get(&g_scene.entity_mapping, id);
}

/**
 * @brief Returns the active contacts of a physics object to the global scene's free contact list.
 *
 * This function iterates through the active contacts of the given physics object and appends
 * them to the global scene's free contact list. It ensures that the active contacts of the
 * object are properly returned and the object's active contact list is cleared.
 *
 * @param object Pointer to the physics object whose active contacts are to be returned.
 */
void collision_scene_release_object_contacts(physics_object* object) {
    contact* last_contact = object->active_contacts;

    while (last_contact && last_contact->next) {
        last_contact = last_contact->next;
    }

    if (last_contact) {
        last_contact->next = g_scene.next_free_contact;
        g_scene.next_free_contact = object->active_contacts;
        object->active_contacts = NULL;
    }
}

/// @brief Removes a physics object from the collision scene. 
///
/// The removed object will no longer be updated in the phys loop or considered for collision.
/// All contacts associated with the object will be released.
/// @param object 
void collision_scene_remove(physics_object* object) {
    bool has_found = false;

    for (int i = 0; i < g_scene.objectCount; i++) {
        if (object == g_scene.elements[i].object) {
            collision_scene_release_object_contacts(object);
            has_found = true;
        }

        if (has_found &&  i + 1 < g_scene.objectCount) {
            g_scene.elements[i] = g_scene.elements[i + 1];
        }
    }

    if (has_found) {
        g_scene.objectCount -= 1;
    }
    AABB_tree_remove_leaf_node(&g_scene.object_aabbtree, object->_aabb_tree_node_id, true);
    hash_map_delete(&g_scene.entity_mapping, object->entity_id);
}


void collision_scene_use_static_collision(struct mesh_collider* mesh_collider) {
    g_scene.mesh_collider = mesh_collider;
}


/// @brief Removes the current static collision mesh from the scene.
void collision_scene_remove_static_collision() {
    AABB_tree_free(&g_scene.mesh_collider->aabbtree);
    g_scene.mesh_collider = NULL;
}


/// @brief Performs collision detection for a single physics object against other objects in the scene.
///
/// First a broad phase detection using a AABB BVH tree is performed to find potential collision candidates.
/// Then a narrow phase detection (GJK/EPA) is performed to check for and resolve actual collisions.
/// @param element the scene element (phys object) to check for collisions
void collision_scene_collide_phys_object(struct collision_scene_element* element) {

    //perform a broad phase check to find potential collision candidates by traversing the AABB BVH tree
    //and collecting leaf nodes that overlap with the object's bounding box
    int result_count = 0;
    int max_results = 10;
    node_proxy results[max_results];
    AABB_tree_query_bounds(&g_scene.object_aabbtree, &element->object->bounding_box, results, &result_count, max_results);

    //iterate over the list of candidates and perform detailed collision detection
    for (size_t i = 0; i < result_count; i++)
    {
        physics_object *other = (physics_object *)AABB_tree_get_node_data(&g_scene.object_aabbtree, results[i]);
        // skip narrow phase if there is no physics object associated with the result node
        // or if it is the same object as the one queried
        if (!other || other == element->object)
        {
            continue;
        }
        collide_object_to_object(element->object, other);

    }
}

#define MAX_SWEPT_ITERATIONS    5

void collision_scene_collide_object_to_static(physics_object* object, Vector3* prev_pos) {

    for (int i = 0; i < MAX_SWEPT_ITERATIONS; i += 1)
    {
        Vector3 displacement;
        vector3FromTo(prev_pos, object->position, &displacement);
        Vector3 bounding_box_size;
        vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &bounding_box_size);
        vector3Scale(&bounding_box_size, &bounding_box_size, 0.5f);

        // if the object has moved more than the bounding box size perform a swept collision check
        if (fabs(displacement.x) > bounding_box_size.x ||
            fabs(displacement.y) > bounding_box_size.y ||
            fabs(displacement.z) > bounding_box_size.z)
        {
            if (!collide_object_to_mesh_swept(object, g_scene.mesh_collider, prev_pos))
            {
                return;
            }
        }
        // otherwise just do a normal collision check
        else
        {
            collide_object_to_mesh(object, g_scene.mesh_collider);

            return;
        }
    }

    // too many swept iterations
    // to prevent tunneling just move the object back
    // to the previous known valid location
    *object->position = *prev_pos;
}


// ============================================================================
// NEW: ITERATIVE CONSTRAINT SOLVER PHASES
// ============================================================================

/// @brief Helper function to calculate tangent vectors orthogonal to normal
static void calculate_tangent_vectors(const Vector3* normal, Vector3* tangent_u, Vector3* tangent_v) {
    // Choose an axis that's not parallel to the normal
    Vector3 axis;
    if (fabsf(normal->x) > 0.9f) {
        axis = (Vector3){{0.0f, 1.0f, 0.0f}};
    } else {
        axis = (Vector3){{1.0f, 0.0f, 0.0f}};
    }

    // First tangent is normal cross axis
    vector3Cross(normal, &axis, tangent_u);
    vector3Normalize(tangent_u, tangent_u);

    // Second tangent is normal cross first tangent
    vector3Cross(normal, tangent_u, tangent_v);
    vector3Normalize(tangent_v, tangent_v);
}

/// @brief Mark all cached contacts as inactive before detection phase
static void collision_scene_mark_contacts_inactive() {
    for (int i = 0; i < g_scene.cached_contact_count; i++) {
        g_scene.cached_contacts[i].is_active = false;
    }
}

/// @brief Remove inactive contacts and compact the array
static void collision_scene_remove_inactive_contacts() {
    int write_index = 0;
    for (int read_index = 0; read_index < g_scene.cached_contact_count; read_index++) {
        if (g_scene.cached_contacts[read_index].is_active) {
            if (write_index != read_index) {
                g_scene.cached_contacts[write_index] = g_scene.cached_contacts[read_index];
            }
            write_index++;
        }
    }
    g_scene.cached_contact_count = write_index;
}

/// @brief Detect all contacts (object-to-object and object-to-mesh)
static void collision_scene_detect_all_contacts() {
    // Mark all existing contacts as inactive
    collision_scene_mark_contacts_inactive();

    // Detect object-to-object collisions
    for (int i = 0; i < g_scene.objectCount; i++) {
        physics_object* a = g_scene.elements[i].object;

        if (a->_is_sleeping) continue;

        // Broad phase
        int result_count = 0;
        int max_results = 10;
        node_proxy results[max_results];
        AABB_tree_query_bounds(&g_scene.object_aabbtree, &a->bounding_box,
                               results, &result_count, max_results);

        for (int j = 0; j < result_count; j++) {
            physics_object* b = (physics_object*)AABB_tree_get_node_data(
                &g_scene.object_aabbtree, results[j]);

            if (!b || b == a) continue;

            // Narrow phase - only detect, don't resolve!
            detect_contact_object_to_object(a, b);
        }
    }

    // Detect object-to-mesh collisions
    if (g_scene.mesh_collider) {
        for (int i = 0; i < g_scene.objectCount; i++) {
            physics_object* obj = g_scene.elements[i].object;

            // Detect mesh collision for all non-sleeping, non-kinematic, tangible objects
            // We need to check even stationary objects because they might have gravity or be resting on the mesh
            if (!obj->_is_sleeping && !obj->is_kinematic &&
                (obj->collision_layers & COLLISION_LAYER_TANGIBLE)) {
                detect_contacts_object_to_mesh(obj, g_scene.mesh_collider);
            }
        }
    }

    // Remove contacts that were not detected this frame
    collision_scene_remove_inactive_contacts();
}

/// @brief Pre-solve: calculate effective masses and prepare constraint data
static void collision_scene_pre_solve_contacts() {
    for (int i = 0; i < g_scene.cached_contact_count; i++) {
        contact_constraint* cc = &g_scene.cached_contacts[i];

        if (!cc->is_active || cc->is_trigger) continue;

        physics_object* a = cc->objectA;
        physics_object* b = cc->objectB;

        // Calculate center of mass for both objects
        Vector3 centerOfMassA = gZeroVec;
        Vector3 centerOfMassB = gZeroVec;

        if (a) {
            if (a->rotation) {
                Vector3 rotatedOffset;
                quatMultVector(a->rotation, &a->center_offset, &rotatedOffset);
                vector3Add(a->position, &rotatedOffset, &centerOfMassA);
            } else {
                vector3Add(a->position, &a->center_offset, &centerOfMassA);
            }
        }

        if (b) {
            if (b->rotation) {
                Vector3 rotatedOffset;
                quatMultVector(b->rotation, &b->center_offset, &rotatedOffset);
                vector3Add(b->position, &rotatedOffset, &centerOfMassB);
            } else {
                vector3Add(b->position, &b->center_offset, &centerOfMassB);
            }
        }

        // Calculate rA and rB (contact point relative to center of mass)
        // IMPORTANT: For static mesh (entity_a = 0), contactA is on mesh surface, contactB is on object
        // For object-object, contactA is on A, contactB is on B
        if (a) {
            // A is a physics object, use contactA
            vector3Sub(&cc->contactA, &centerOfMassA, &cc->rA);
        } else {
            // A is static mesh (entity_a = 0), no center of mass, rA is zero
            cc->rA = gZeroVec;
        }

        if (b) {
            // B is a physics object, use contactB
            vector3Sub(&cc->contactB, &centerOfMassB, &cc->rB);
        } else {
            // This shouldn't happen (B is always an object), but handle it
            cc->rB = gZeroVec;
        }

        // Calculate tangent vectors for friction
        calculate_tangent_vectors(&cc->normal, &cc->tangent_u, &cc->tangent_v);

        // Calculate effective mass for normal direction (same as denominator in correct_velocity)
        // This is the full logic from correct_velocity preserving all constraint handling

        bool aMovementConstrained = a && (a->is_kinematic || ((a->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));
        bool bMovementConstrained = b && (b->is_kinematic || ((b->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));

        float invMassA = 0.0f;
        float invMassB = 0.0f;

        if (a && !aMovementConstrained) {
            bool constrainedAlongNormal = ((a->constraints & CONSTRAINTS_FREEZE_POSITION_X) && fabsf(cc->normal.x) > 0.01f) ||
                                          ((a->constraints & CONSTRAINTS_FREEZE_POSITION_Y) && fabsf(cc->normal.y) > 0.01f) ||
                                          ((a->constraints & CONSTRAINTS_FREEZE_POSITION_Z) && fabsf(cc->normal.z) > 0.01f);
            invMassA = constrainedAlongNormal ? 0.0f : a->_inv_mass;
        }

        if (b && !bMovementConstrained) {
            bool constrainedAlongNormal = ((b->constraints & CONSTRAINTS_FREEZE_POSITION_X) && fabsf(cc->normal.x) > 0.01f) ||
                                          ((b->constraints & CONSTRAINTS_FREEZE_POSITION_Y) && fabsf(cc->normal.y) > 0.01f) ||
                                          ((b->constraints & CONSTRAINTS_FREEZE_POSITION_Z) && fabsf(cc->normal.z) > 0.01f);
            invMassB = constrainedAlongNormal ? 0.0f : b->_inv_mass;
        }

        float denominator = invMassA + invMassB;

        // Add rotational inertia term for A (from correct_velocity)
        if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
            Vector3 rCrossN;
            vector3Cross(&cc->rA, &cc->normal, &rCrossN);

            Quaternion rotation_inverse_a;
            quatConjugate(a->rotation, &rotation_inverse_a);

            Vector3 local_rCrossN;
            quatMultVector(&rotation_inverse_a, &rCrossN, &local_rCrossN);

            Vector3 local_torquePerImpulse;
            local_torquePerImpulse.x = local_rCrossN.x * a->_inv_local_intertia_tensor.x;
            local_torquePerImpulse.y = local_rCrossN.y * a->_inv_local_intertia_tensor.y;
            local_torquePerImpulse.z = local_rCrossN.z * a->_inv_local_intertia_tensor.z;

            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;

            Vector3 torquePerImpulse;
            quatMultVector(a->rotation, &local_torquePerImpulse, &torquePerImpulse);

            denominator += vector3Dot(&rCrossN, &torquePerImpulse);
        }

        // Add rotational inertia term for B
        if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
            Vector3 rCrossN;
            vector3Cross(&cc->rB, &cc->normal, &rCrossN);

            Quaternion rotation_inverse_b;
            quatConjugate(b->rotation, &rotation_inverse_b);

            Vector3 local_rCrossN;
            quatMultVector(&rotation_inverse_b, &rCrossN, &local_rCrossN);

            Vector3 local_torquePerImpulse;
            local_torquePerImpulse.x = local_rCrossN.x * b->_inv_local_intertia_tensor.x;
            local_torquePerImpulse.y = local_rCrossN.y * b->_inv_local_intertia_tensor.y;
            local_torquePerImpulse.z = local_rCrossN.z * b->_inv_local_intertia_tensor.z;

            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;

            Vector3 torquePerImpulse;
            quatMultVector(b->rotation, &local_torquePerImpulse, &torquePerImpulse);

            denominator += vector3Dot(&rCrossN, &torquePerImpulse);
        }

        if (denominator < EPSILON) denominator = EPSILON;
        cc->normal_mass = 1.0f / denominator;

        // Calculate effective mass for tangent directions (for friction)
        // We need to calculate this properly for each tangent direction

        // Tangent U effective mass
        float denominator_u = invMassA + invMassB;
        if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
            Vector3 rCrossT;
            vector3Cross(&cc->rA, &cc->tangent_u, &rCrossT);
            Quaternion rotation_inverse_a;
            quatConjugate(a->rotation, &rotation_inverse_a);
            Vector3 local_rCrossT;
            quatMultVector(&rotation_inverse_a, &rCrossT, &local_rCrossT);
            Vector3 local_torquePerImpulse;
            local_torquePerImpulse.x = local_rCrossT.x * a->_inv_local_intertia_tensor.x;
            local_torquePerImpulse.y = local_rCrossT.y * a->_inv_local_intertia_tensor.y;
            local_torquePerImpulse.z = local_rCrossT.z * a->_inv_local_intertia_tensor.z;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;
            Vector3 torquePerImpulse;
            quatMultVector(a->rotation, &local_torquePerImpulse, &torquePerImpulse);
            denominator_u += vector3Dot(&rCrossT, &torquePerImpulse);
        }
        if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
            Vector3 rCrossT;
            vector3Cross(&cc->rB, &cc->tangent_u, &rCrossT);
            Quaternion rotation_inverse_b;
            quatConjugate(b->rotation, &rotation_inverse_b);
            Vector3 local_rCrossT;
            quatMultVector(&rotation_inverse_b, &rCrossT, &local_rCrossT);
            Vector3 local_torquePerImpulse;
            local_torquePerImpulse.x = local_rCrossT.x * b->_inv_local_intertia_tensor.x;
            local_torquePerImpulse.y = local_rCrossT.y * b->_inv_local_intertia_tensor.y;
            local_torquePerImpulse.z = local_rCrossT.z * b->_inv_local_intertia_tensor.z;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;
            Vector3 torquePerImpulse;
            quatMultVector(b->rotation, &local_torquePerImpulse, &torquePerImpulse);
            denominator_u += vector3Dot(&rCrossT, &torquePerImpulse);
        }
        if (denominator_u < EPSILON) denominator_u = EPSILON;
        cc->tangent_mass_u = 1.0f / denominator_u;

        // Tangent V effective mass
        float denominator_v = invMassA + invMassB;
        if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
            Vector3 rCrossT;
            vector3Cross(&cc->rA, &cc->tangent_v, &rCrossT);
            Quaternion rotation_inverse_a;
            quatConjugate(a->rotation, &rotation_inverse_a);
            Vector3 local_rCrossT;
            quatMultVector(&rotation_inverse_a, &rCrossT, &local_rCrossT);
            Vector3 local_torquePerImpulse;
            local_torquePerImpulse.x = local_rCrossT.x * a->_inv_local_intertia_tensor.x;
            local_torquePerImpulse.y = local_rCrossT.y * a->_inv_local_intertia_tensor.y;
            local_torquePerImpulse.z = local_rCrossT.z * a->_inv_local_intertia_tensor.z;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;
            Vector3 torquePerImpulse;
            quatMultVector(a->rotation, &local_torquePerImpulse, &torquePerImpulse);
            denominator_v += vector3Dot(&rCrossT, &torquePerImpulse);
        }
        if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
            Vector3 rCrossT;
            vector3Cross(&cc->rB, &cc->tangent_v, &rCrossT);
            Quaternion rotation_inverse_b;
            quatConjugate(b->rotation, &rotation_inverse_b);
            Vector3 local_rCrossT;
            quatMultVector(&rotation_inverse_b, &rCrossT, &local_rCrossT);
            Vector3 local_torquePerImpulse;
            local_torquePerImpulse.x = local_rCrossT.x * b->_inv_local_intertia_tensor.x;
            local_torquePerImpulse.y = local_rCrossT.y * b->_inv_local_intertia_tensor.y;
            local_torquePerImpulse.z = local_rCrossT.z * b->_inv_local_intertia_tensor.z;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_torquePerImpulse.x = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_torquePerImpulse.y = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_torquePerImpulse.z = 0.0f;
            Vector3 torquePerImpulse;
            quatMultVector(b->rotation, &local_torquePerImpulse, &torquePerImpulse);
            denominator_v += vector3Dot(&rCrossT, &torquePerImpulse);
        }
        if (denominator_v < EPSILON) denominator_v = EPSILON;
        cc->tangent_mass_v = 1.0f / denominator_v;
    }
}

/// @brief Warm start: apply accumulated impulses from previous frame
static void collision_scene_warm_start() {
    for (int i = 0; i < g_scene.cached_contact_count; i++) {
        contact_constraint* cc = &g_scene.cached_contacts[i];

        if (!cc->is_active || cc->is_trigger) continue;

        physics_object* a = cc->objectA;
        physics_object* b = cc->objectB;

        // Apply accumulated normal impulse
        Vector3 impulse;
        vector3Scale(&cc->normal, &impulse, cc->accumulated_normal_impulse);

        // Apply to object A
        if (a && !a->is_kinematic) {
            Vector3 linearImpulse;
            vector3Scale(&impulse, &linearImpulse, a->_inv_mass);

            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X)) a->velocity.x += linearImpulse.x;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) a->velocity.y += linearImpulse.y;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) a->velocity.z += linearImpulse.z;

            if (a->rotation) {
                Vector3 angularImpulse;
                vector3Cross(&cc->rA, &impulse, &angularImpulse);
                physics_object_apply_angular_impulse(a, &angularImpulse);
            }
        }

        // Apply to object B (opposite direction)
        if (b && !b->is_kinematic) {
            Vector3 linearImpulse;
            vector3Scale(&impulse, &linearImpulse, -b->_inv_mass);

            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X)) b->velocity.x += linearImpulse.x;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) b->velocity.y += linearImpulse.y;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) b->velocity.z += linearImpulse.z;

            if (b->rotation) {
                Vector3 angularImpulse;
                vector3Cross(&cc->rB, &impulse, &angularImpulse);
                vector3Negate(&angularImpulse, &angularImpulse);
                physics_object_apply_angular_impulse(b, &angularImpulse);
            }
        }

        // Apply accumulated tangent impulses for friction
        Vector3 tangentImpulseU;
        vector3Scale(&cc->tangent_u, &tangentImpulseU, cc->accumulated_tangent_impulse_u);

        Vector3 tangentImpulseV;
        vector3Scale(&cc->tangent_v, &tangentImpulseV, cc->accumulated_tangent_impulse_v);

        // Apply tangent_u to A
        if (a && !a->is_kinematic) {
            Vector3 linearTangentU;
            vector3Scale(&tangentImpulseU, &linearTangentU, a->_inv_mass);
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X)) a->velocity.x += linearTangentU.x;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) a->velocity.y += linearTangentU.y;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) a->velocity.z += linearTangentU.z;

            if (a->rotation) {
                Vector3 angularTangentU;
                vector3Cross(&cc->rA, &tangentImpulseU, &angularTangentU);
                physics_object_apply_angular_impulse(a, &angularTangentU);
            }
        }

        // Apply tangent_u to B (opposite)
        if (b && !b->is_kinematic) {
            Vector3 linearTangentU;
            vector3Scale(&tangentImpulseU, &linearTangentU, -b->_inv_mass);
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X)) b->velocity.x += linearTangentU.x;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) b->velocity.y += linearTangentU.y;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) b->velocity.z += linearTangentU.z;

            if (b->rotation) {
                Vector3 angularTangentU;
                vector3Cross(&cc->rB, &tangentImpulseU, &angularTangentU);
                vector3Negate(&angularTangentU, &angularTangentU);
                physics_object_apply_angular_impulse(b, &angularTangentU);
            }
        }

        // Apply tangent_v to A
        if (a && !a->is_kinematic) {
            Vector3 linearTangentV;
            vector3Scale(&tangentImpulseV, &linearTangentV, a->_inv_mass);
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X)) a->velocity.x += linearTangentV.x;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) a->velocity.y += linearTangentV.y;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) a->velocity.z += linearTangentV.z;

            if (a->rotation) {
                Vector3 angularTangentV;
                vector3Cross(&cc->rA, &tangentImpulseV, &angularTangentV);
                physics_object_apply_angular_impulse(a, &angularTangentV);
            }
        }

        // Apply tangent_v to B (opposite)
        if (b && !b->is_kinematic) {
            Vector3 linearTangentV;
            vector3Scale(&tangentImpulseV, &linearTangentV, -b->_inv_mass);
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X)) b->velocity.x += linearTangentV.x;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) b->velocity.y += linearTangentV.y;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) b->velocity.z += linearTangentV.z;

            if (b->rotation) {
                Vector3 angularTangentV;
                vector3Cross(&cc->rB, &tangentImpulseV, &angularTangentV);
                vector3Negate(&angularTangentV, &angularTangentV);
                physics_object_apply_angular_impulse(b, &angularTangentV);
            }
        }
    }
}

/// @brief Solve velocity constraints iteratively
static void collision_scene_solve_velocity_constraints() {
    const float slop = 0.005f;
    const float bounceThreshold = 0.5f;

    for (int i = 0; i < g_scene.cached_contact_count; i++) {
        contact_constraint* cc = &g_scene.cached_contacts[i];

        if (!cc->is_active || cc->is_trigger) continue;

        physics_object* a = cc->objectA;
        physics_object* b = cc->objectB;

        // Calculate contact velocities
        Vector3 contactVelA = gZeroVec;
        Vector3 contactVelB = gZeroVec;

        if (a && !a->is_kinematic) {
            contactVelA = a->velocity;
            if (a->rotation) {
                Vector3 angularContribution;
                vector3Cross(&a->angular_velocity, &cc->rA, &angularContribution);
                vector3Add(&contactVelA, &angularContribution, &contactVelA);
            }
        }

        if (b && !b->is_kinematic) {
            contactVelB = b->velocity;
            if (b->rotation) {
                Vector3 angularContribution;
                vector3Cross(&b->angular_velocity, &cc->rB, &angularContribution);
                vector3Add(&contactVelB, &angularContribution, &contactVelB);
            }
        }

        // Calculate relative velocity
        Vector3 relVel;
        vector3Sub(&contactVelA, &contactVelB, &relVel);
        float normalVelocity = vector3Dot(&relVel, &cc->normal);

        // Compute bounce with damping for low-velocity contacts
        float effectiveBounce = cc->combined_bounce;
        if (fabsf(normalVelocity) < bounceThreshold) {
            effectiveBounce = cc->combined_bounce * (fabsf(normalVelocity) / bounceThreshold);
        }

        // Add Baumgarte stabilization bias to prevent penetration
        const float baumgarteFactor = 0.2f; // Conservative bias
        float baumgarteBias = 0.0f;
        if (cc->penetration > slop) {
            baumgarteBias = (baumgarteFactor / FIXED_DELTATIME) * (cc->penetration - slop);
        }

        // Calculate lambda (impulse change)
        float lambda = -(normalVelocity * (1.0f + effectiveBounce) - baumgarteBias) * cc->normal_mass;

        // Clamp accumulated impulse (key difference from old approach!)
        float oldImpulse = cc->accumulated_normal_impulse;
        cc->accumulated_normal_impulse = maxf(oldImpulse + lambda, 0.0f);
        lambda = cc->accumulated_normal_impulse - oldImpulse;

        if (fabsf(lambda) < EPSILON) continue;

        // Apply lambda (the change, not total)
        Vector3 impulse;
        vector3Scale(&cc->normal, &impulse, lambda);

        // Apply to object A
        if (a && !a->is_kinematic) {
            Vector3 linearImpulse;
            vector3Scale(&impulse, &linearImpulse, a->_inv_mass);

            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X)) a->velocity.x += linearImpulse.x;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) a->velocity.y += linearImpulse.y;
            if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) a->velocity.z += linearImpulse.z;

            if (a->rotation) {
                Vector3 angularImpulse;
                vector3Cross(&cc->rA, &impulse, &angularImpulse);
                physics_object_apply_angular_impulse(a, &angularImpulse);
            }
        }

        // Apply to object B
        if (b && !b->is_kinematic) {
            Vector3 linearImpulse;
            vector3Scale(&impulse, &linearImpulse, -b->_inv_mass);

            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X)) b->velocity.x += linearImpulse.x;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) b->velocity.y += linearImpulse.y;
            if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) b->velocity.z += linearImpulse.z;

            if (b->rotation) {
                Vector3 angularImpulse;
                vector3Cross(&cc->rB, &impulse, &angularImpulse);
                vector3Negate(&angularImpulse, &angularImpulse);
                physics_object_apply_angular_impulse(b, &angularImpulse);
            }
        }

        #ifndef DEBUG_IGNORE_FRICTION
        // Handle friction with proper accumulation
        if (cc->combined_friction > 0.0f) {
            // Recalculate relative velocity after normal impulse
            if (a && !a->is_kinematic) {
                contactVelA = a->velocity;
                if (a->rotation) {
                    Vector3 angularContribution;
                    vector3Cross(&a->angular_velocity, &cc->rA, &angularContribution);
                    vector3Add(&contactVelA, &angularContribution, &contactVelA);
                }
            }

            if (b && !b->is_kinematic) {
                contactVelB = b->velocity;
                if (b->rotation) {
                    Vector3 angularContribution;
                    vector3Cross(&b->angular_velocity, &cc->rB, &angularContribution);
                    vector3Add(&contactVelB, &angularContribution, &contactVelB);
                }
            }

            vector3Sub(&contactVelA, &contactVelB, &relVel);

            // Calculate tangential velocity components along tangent_u and tangent_v
            float vTangentU = vector3Dot(&relVel, &cc->tangent_u);
            float vTangentV = vector3Dot(&relVel, &cc->tangent_v);

            // Calculate friction impulse changes (lambda) for both tangent directions
            float lambdaU = -vTangentU * cc->tangent_mass_u;
            float lambdaV = -vTangentV * cc->tangent_mass_v;

            // Calculate new accumulated tangent impulses
            float newAccumU = cc->accumulated_tangent_impulse_u + lambdaU;
            float newAccumV = cc->accumulated_tangent_impulse_v + lambdaV;

            // Clamp to friction cone (Coulomb's law: |tangent_impulse| <= friction * normal_impulse)
            float maxFriction = cc->combined_friction * cc->accumulated_normal_impulse;
            float tangentMagnitude = sqrtf(newAccumU * newAccumU + newAccumV * newAccumV);

            if (tangentMagnitude > maxFriction) {
                float scale = maxFriction / tangentMagnitude;
                newAccumU *= scale;
                newAccumV *= scale;
            }

            // Calculate actual impulse deltas to apply
            lambdaU = newAccumU - cc->accumulated_tangent_impulse_u;
            lambdaV = newAccumV - cc->accumulated_tangent_impulse_v;

            // Update accumulated values
            cc->accumulated_tangent_impulse_u = newAccumU;
            cc->accumulated_tangent_impulse_v = newAccumV;

            // Apply tangent_u impulse
            if (fabsf(lambdaU) > EPSILON) {
                Vector3 tangentImpulseU;
                vector3Scale(&cc->tangent_u, &tangentImpulseU, lambdaU);

                if (a && !a->is_kinematic) {
                    Vector3 linearImpulse;
                    vector3Scale(&tangentImpulseU, &linearImpulse, a->_inv_mass);
                    if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X)) a->velocity.x += linearImpulse.x;
                    if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) a->velocity.y += linearImpulse.y;
                    if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) a->velocity.z += linearImpulse.z;

                    if (a->rotation) {
                        Vector3 angularImpulse;
                        vector3Cross(&cc->rA, &tangentImpulseU, &angularImpulse);
                        physics_object_apply_angular_impulse(a, &angularImpulse);
                    }
                }

                if (b && !b->is_kinematic) {
                    Vector3 linearImpulse;
                    vector3Scale(&tangentImpulseU, &linearImpulse, -b->_inv_mass);
                    if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X)) b->velocity.x += linearImpulse.x;
                    if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) b->velocity.y += linearImpulse.y;
                    if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) b->velocity.z += linearImpulse.z;

                    if (b->rotation) {
                        Vector3 angularImpulse;
                        vector3Cross(&cc->rB, &tangentImpulseU, &angularImpulse);
                        vector3Negate(&angularImpulse, &angularImpulse);
                        physics_object_apply_angular_impulse(b, &angularImpulse);
                    }
                }
            }

            // Apply tangent_v impulse
            if (fabsf(lambdaV) > EPSILON) {
                Vector3 tangentImpulseV;
                vector3Scale(&cc->tangent_v, &tangentImpulseV, lambdaV);

                if (a && !a->is_kinematic) {
                    Vector3 linearImpulse;
                    vector3Scale(&tangentImpulseV, &linearImpulse, a->_inv_mass);
                    if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X)) a->velocity.x += linearImpulse.x;
                    if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) a->velocity.y += linearImpulse.y;
                    if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) a->velocity.z += linearImpulse.z;

                    if (a->rotation) {
                        Vector3 angularImpulse;
                        vector3Cross(&cc->rA, &tangentImpulseV, &angularImpulse);
                        physics_object_apply_angular_impulse(a, &angularImpulse);
                    }
                }

                if (b && !b->is_kinematic) {
                    Vector3 linearImpulse;
                    vector3Scale(&tangentImpulseV, &linearImpulse, -b->_inv_mass);
                    if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X)) b->velocity.x += linearImpulse.x;
                    if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)) b->velocity.y += linearImpulse.y;
                    if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)) b->velocity.z += linearImpulse.z;

                    if (b->rotation) {
                        Vector3 angularImpulse;
                        vector3Cross(&cc->rB, &tangentImpulseV, &angularImpulse);
                        vector3Negate(&angularImpulse, &angularImpulse);
                        physics_object_apply_angular_impulse(b, &angularImpulse);
                    }
                }
            }
        }
        #endif
    }
}

/// @brief Solve position constraints iteratively
static void collision_scene_solve_position_constraints() {
    const float slop = 0.005f;
    const float steeringConstant = 0.2f;
    const float maxCorrection = 0.08f;

    for (int i = 0; i < g_scene.cached_contact_count; i++) {
        contact_constraint* cc = &g_scene.cached_contacts[i];

        if (!cc->is_active || cc->is_trigger) continue;
        if (cc->penetration < slop) continue;

        physics_object* a = cc->objectA;
        physics_object* b = cc->objectB;

        const float steeringForce = clampf(steeringConstant * (cc->penetration + slop), 0, maxCorrection);

        bool aMovementConstrained = a && (a->is_kinematic || ((a->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));
        bool bMovementConstrained = b && (b->is_kinematic || ((b->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));

        float invMassA = 0.0f;
        float invMassB = 0.0f;

        Vector3 effectiveNormalA = cc->normal;
        Vector3 effectiveNormalB = cc->normal;

        float normal_dot_inv = 1.0f / vector3Dot(&cc->normal, &cc->normal);

        if (a && !aMovementConstrained) {
            if (a->constraints & CONSTRAINTS_FREEZE_POSITION_X)
                effectiveNormalA.x = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)
                effectiveNormalA.y = 0.0f;
            if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)
                effectiveNormalA.z = 0.0f;

            float normalDotA = vector3Dot(&effectiveNormalA, &cc->normal);
            invMassA = a->_inv_mass * (normalDotA * normalDotA) * normal_dot_inv;
        }

        if (b && !bMovementConstrained) {
            if (b->constraints & CONSTRAINTS_FREEZE_POSITION_X)
                effectiveNormalB.x = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Y)
                effectiveNormalB.y = 0.0f;
            if (b->constraints & CONSTRAINTS_FREEZE_POSITION_Z)
                effectiveNormalB.z = 0.0f;

            float normalDotB = vector3Dot(&effectiveNormalB, &cc->normal);
            invMassB = b->_inv_mass * (normalDotB * normalDotB) * normal_dot_inv;
        }

        float invMassSum = invMassA + invMassB;
        if (invMassSum == 0.0f) continue;

        float correctionMag = steeringForce / invMassSum;

        // Apply correction
        if (a && invMassA > 0.0f) {
            vector3AddScaled(a->position, &effectiveNormalA, correctionMag * invMassA, a->position);
        }

        if (b && invMassB > 0.0f) {
            vector3AddScaled(b->position, &effectiveNormalB, -correctionMag * invMassB, b->position);
        }
    }
}

/// @brief performs a physics step on all objects in the scene using iterative constraint solver
void collision_scene_step() {
    struct collision_scene_element* element;

    // ========================================================================
    // PHASE 1: Apply gravity and integrate velocities
    // ========================================================================
    for (int i = 0; i < g_scene.objectCount; i++) {
        element = &g_scene.elements[i];
        physics_object* obj = element->object;
        obj->_ground_support_factor = 0;

        if (!obj->_is_sleeping && obj->has_gravity && !obj->is_kinematic)
        {
            // Check if object has ground contact (using old contact list)
            float ground_support_factor = 0.0f;
            if (obj->active_contacts)
            {
                contact *contact = obj->active_contacts;
                while (contact)
                {
                    // Determine the contact normal pointing most up
                    if (contact->normal.y > ground_support_factor && contact->other_object != 0)
                    {
                        ground_support_factor = contact->normal.y;
                        break;
                    }
                    contact = contact->next;
                }
            }
            obj->_ground_support_factor = ground_support_factor;
            obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar;
        }

        // Release old contacts from previous frame
        collision_scene_release_object_contacts(obj);

        // Integrate acceleration into velocity (but don't update position yet!)
        physics_object_integrate_velocity(obj);

        // Update angular velocity
        physics_object_update_angular_velocity(obj);
    }

    // ========================================================================
    // PHASE 2: Detect all contacts (without resolving)
    // ========================================================================
    collision_scene_detect_all_contacts();

    // ========================================================================
    // PHASE 3: Pre-solve - calculate effective masses and prepare constraints
    // ========================================================================
    collision_scene_pre_solve_contacts();

    // ========================================================================
    // PHASE 4: Warm start - apply cached impulses from previous frame
    // ========================================================================
    collision_scene_warm_start();

    // ========================================================================
    // PHASE 5: Solve velocity constraints iteratively
    // ========================================================================
    for (int iter = 0; iter < 3; iter++) {
        collision_scene_solve_velocity_constraints();
    }

    // ========================================================================
    // PHASE 6: Integrate positions from velocities and update AABBs
    // ========================================================================
    for (int i = 0; i < g_scene.objectCount; i++) {
        element = &g_scene.elements[i];
        physics_object* obj = element->object;

        // Integrate velocity into position
        physics_object_integrate_position(obj);

        // Recalculate AABB if object is awake (position may have changed due to velocity integration or solver)
        if (!obj->_is_sleeping) {
            // Check if object actually moved or rotated this frame
            const bool has_moved = !vector3IsIdentical(&obj->_prev_step_pos, obj->position);
            const bool has_rotated = obj->rotation ? !quatIsIdentical(obj->rotation, &obj->_prev_step_rot) : false;

            if (has_moved || has_rotated) {
                physics_object_recalculate_aabb(obj);
                Vector3 displacement;
                vector3FromTo(&obj->_prev_step_pos, obj->position, &displacement);
                AABB_tree_move_node(&g_scene.object_aabbtree, obj->_aabb_tree_node_id,
                                    obj->bounding_box, &displacement);
            }
        }
    }

    // ========================================================================
    // PHASE 7: Solve position constraints iteratively (3 iterations)
    // ========================================================================
    for (int iter = 0; iter < 3; iter++) {
        collision_scene_solve_position_constraints();
    }

    // ========================================================================
    // PHASE 8: Apply position constraints and update sleep states
    // ========================================================================
    g_scene._sleepy_count = 0;
    for (int i = 0; i < g_scene.objectCount; i++) {
        element = &g_scene.elements[i];
        physics_object* obj = element->object;

        // Apply physical constraints to the object
        physics_object_apply_position_constraints(obj);

        // Update sleep state
        // Check for external position changes (non-physics movement)
        const bool position_changed = vector3DistSqrd(obj->position, &obj->_prev_step_pos) > PHYS_OBJECT_POS_CHANGE_SLEEP_THRESHOLD_SQ;

        // Check for external rotation changes (non-physics rotation)
        bool rotation_changed = false;
        if (obj->rotation)
        {
            float quat_similarity = fabsf(quatDot(obj->rotation, &obj->_prev_step_rot));
            rotation_changed = quat_similarity < PHYS_OBJECT_ROT_SIMILARITY_SLEEP_THRESHOLD;
        }

        // Check physics-driven motion via velocities
        const bool has_linear_velocity = vector3MagSqrd(&obj->velocity) > PHYS_OBJECT_SPEED_SLEEP_THRESHOLD_SQ;
        const bool has_angular_velocity = obj->rotation &&
                                        vector3MagSqrd(&obj->angular_velocity) > PHYS_OBJECT_ANGULAR_CHANGE_SLEEP_THRESHOLD_SQ;

        // Object is at rest if: no external changes AND no physics velocities
        const bool is_at_rest = !position_changed && !rotation_changed &&
                                !has_linear_velocity && !has_angular_velocity;

        if (is_at_rest)
        {
            if (obj->_sleep_counter < PHYS_OBJECT_SLEEP_STEPS)
            {
                obj->_sleep_counter += 1;
            }
            else
            {
                obj->_is_sleeping = true;
                // Get rid of any residual velocity so object is actually at rest
                obj->velocity = gZeroVec;
                obj->angular_velocity = gZeroVec;
            }
        }
        else
        {
            obj->_is_sleeping = false;
            obj->_sleep_counter = 0;
        }

        if(!obj->_is_sleeping){
            // Only update the previous position if the object is awake
            obj->_prev_step_pos = *obj->position;
            if (obj->rotation)
            {
                obj->_prev_step_rot = *obj->rotation;
            }
        }
        else{
            g_scene._sleepy_count += 1;
        }
    }
}

/// @brief Returns a new contact from the global scene's free contact list, NULL if none are available.
contact* collision_scene_new_contact() {
    if (!g_scene.next_free_contact) {
        return NULL;
    }

    contact* result = g_scene.next_free_contact;
    g_scene.next_free_contact = result->next;
    return result;
}