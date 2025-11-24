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
    free(g_scene.cached_contact_constraints);
    AABB_tree_free(&g_scene.object_aabbtree);
    hash_map_destroy(&g_scene.entity_mapping);
    hash_map_destroy(&g_scene.contact_map);

    hash_map_init(&g_scene.entity_mapping, MAX_PHYSICS_OBJECTS);
    hash_map_init(&g_scene.contact_map, MAX_CACHED_CONTACTS);
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
    g_scene.cached_contact_constraints = malloc(sizeof(contact_constraint) * MAX_CACHED_CONTACTS);
    g_scene.cached_contact_constraint_count = 0;
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
    if (!object->_is_sleeping) {
        contact* last_contact = object->active_contacts;

        while (last_contact && last_contact->next) {
            last_contact = last_contact->next;
        }

        if (last_contact) {
            last_contact->next = g_scene.next_free_contact;
            g_scene.next_free_contact = object->active_contacts;
            object->active_contacts = NULL;
        }
    } else {
        contact** pp = &object->active_contacts;
        while (*pp) {
            contact* c = *pp;
            physics_object* other = c->other_object;
            
            // If other object is awake, it will perform collision detection and re-add the contact.
            // So we must remove it here to avoid duplicates.
            // If other object is sleeping (or static), no detection will happen, so we keep the contact.
            if (other && !other->_is_sleeping) {
                *pp = c->next;
                c->next = g_scene.next_free_contact;
                g_scene.next_free_contact = c;
            } else {
                pp = &c->next;
            }
        }
    }
}

/// @brief recursively wake up connected objects so they can react to a change in one of their neighbors
/// @param obj 
static void collision_scene_wake_island(physics_object* obj) {
    if (obj->_is_sleeping) {
        physics_object_wake(obj);
        // Recurse on ALL contacts
        contact* c = obj->active_contacts;
        while (c) {
            if (c->other_object) collision_scene_wake_island(c->other_object);
            c = c->next;
        }
    } else {
        // If already awake, only recurse on SLEEPING contacts to avoid cycles
        contact* c = obj->active_contacts;
        while (c) {
            if (c->other_object && c->other_object->_is_sleeping) {
                collision_scene_wake_island(c->other_object);
            }
            c = c->next;
        }
    }
}

/// @brief Removes a physics object from the collision scene. 
///
/// The removed object will no longer be updated in the phys loop or considered for collision.
/// All contacts associated with the object will be released.
/// @param object 
void collision_scene_remove(physics_object* object) {
    if(!collision_scene_find_object(object->entity_id))return;

    // Cleanup back-references in neighbors
    contact* c = object->active_contacts;
    while (c) {
        physics_object* neighbor = c->other_object;
        if (neighbor) {
            // Wake up the neighbor so it can react to the removal (e.g. fall if it was resting on this object)
            collision_scene_wake_island(neighbor);

            contact** pp = &neighbor->active_contacts;
            while (*pp) {
                contact* neighbor_c = *pp;
                if (neighbor_c->other_object == object) {
                    *pp = neighbor_c->next;
                    neighbor_c->next = g_scene.next_free_contact;
                    g_scene.next_free_contact = neighbor_c;
                    break;
                }
                pp = &(*pp)->next;
            }
        }
        c = c->next;
    }

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

    // Remove cached constraints involving this object
    int write_index = 0;
    bool constraints_removed = false;
    for (int read_index = 0; read_index < g_scene.cached_contact_constraint_count; read_index++) {
        contact_constraint* constraint = &g_scene.cached_contact_constraints[read_index];
        
        if (constraint->objectA == object || constraint->objectB == object) {
            constraints_removed = true;
            continue; // Skip this constraint (remove it)
        }

        if (write_index != read_index) {
            g_scene.cached_contact_constraints[write_index] = g_scene.cached_contact_constraints[read_index];
        }
        write_index++;
    }
    g_scene.cached_contact_constraint_count = write_index;

    // Rebuild contact map if we removed anything
    if (constraints_removed) {
        hash_map_clear(&g_scene.contact_map);

        for (int i = 0; i < g_scene.cached_contact_constraint_count; i++) {
            contact_constraint* c = &g_scene.cached_contact_constraints[i];
            c->next_same_pid_index = -1;
            
            intptr_t existing_idx_plus_1 = (intptr_t)hash_map_get(&g_scene.contact_map, c->pid);
            
            if (existing_idx_plus_1 != 0) {
                c->next_same_pid_index = (int)existing_idx_plus_1 - 1;
            }
            hash_map_set(&g_scene.contact_map, c->pid, (void*)(intptr_t)(i + 1));
        }
    }
}


void collision_scene_use_static_collision(struct mesh_collider* mesh_collider) {
    g_scene.mesh_collider = mesh_collider;
}


/// @brief Removes the current static collision mesh from the scene.
void collision_scene_remove_static_collision() {
    AABB_tree_free(&g_scene.mesh_collider->aabbtree);
    g_scene.mesh_collider = NULL;
}



// ============================================================================
// NEW: ITERATIVE CONSTRAINT SOLVER PHASES
// ============================================================================

/// @brief Helper function to apply angular impulse directly to rotation (for position solver)
static void physics_object_apply_angular_impulse_to_rotation(physics_object* object, Vector3* angular_impulse) {
    if (object->is_kinematic || !object->rotation) {
        return;
    }

    // Skip if all rotation axes are constrained
    if ((object->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL) {
        return;
    }

    // Calculate rotation change vector (axis * angle) in world space
    // rotation_change = I_world^-1 * angular_impulse
    Vector3 rotation_change;
    physics_object_apply_world_inertia(object, angular_impulse, &rotation_change);

    // Apply rotation change to quaternion
    float angle = vector3Mag(&rotation_change);
    if (angle > EPSILON) {
        Vector3 axis;
        vector3Scale(&rotation_change, &axis, 1.0f / angle);
        Quaternion delta_q;
        quatAxisAngle(&axis, angle, &delta_q);
        
        // Apply delta_q to object->rotation
        // q_new = delta_q * q_old
        Quaternion new_rot;
        quatMultiply(&delta_q, object->rotation, &new_rot);
        quatNormalize(&new_rot, object->rotation);
    }
}

/// @brief Refresh contacts: update world positions from local and mark as inactive
static void collision_scene_refresh_contacts() {
    for (int i = 0; i < g_scene.cached_contact_constraint_count; i++) {
        contact_constraint* constraint = &g_scene.cached_contact_constraints[i];
        
        physics_object* a = constraint->objectA;
        physics_object* b = constraint->objectB;

        // If both objects are sleeping (or static), we should keep the contact active without re-detecting
        bool a_sleeping = !a || a->_is_sleeping;
        bool b_sleeping = !b || b->_is_sleeping;

        if (a_sleeping && b_sleeping) {
             constraint->is_active = true;
        } else {
             constraint->is_active = false;
        }
        
        for (int j = 0; j < constraint->point_count; j++) {
            contact_point* cp = &constraint->points[j];
            cp->active = false;

            // Update world positions from local
            if (a) {
                Vector3 rA = cp->localPointA;
                if (a->rotation )quatMultVector(a->rotation, &rA, &rA);
                vector3Add(a->position, &rA, &cp->contactA);
            } else {
                cp->contactA = cp->localPointA;
            }
            
            if (b) {
                Vector3 rB = cp->localPointB;
                if (b->rotation )quatMultVector(b->rotation, &rB, &rB);
                vector3Add(b->position, &rB, &cp->contactB);
            } else {
                cp->contactB = cp->localPointB;
            }
            
            // Update the main point to be on A (arbitrary choice, consistent with cache_contact_constraint)
            cp->point = cp->contactA;
        }
    }
}

/// @brief Remove inactive contacts and compact the array
static void collision_scene_remove_inactive_contacts() {
    int write_index = 0;
    for (int read_index = 0; read_index < g_scene.cached_contact_constraint_count; read_index++) {
        contact_constraint* constraint = &g_scene.cached_contact_constraints[read_index];
        
        // Prune inactive points
        int point_write_index = 0;
        for (int point_read_index = 0; point_read_index < constraint->point_count; point_read_index++) {
            if (constraint->points[point_read_index].active) {
                if (point_write_index != point_read_index) {
                    constraint->points[point_write_index] = constraint->points[point_read_index];
                }
                point_write_index++;
            }
        }
        constraint->point_count = point_write_index;

        if (constraint->is_active && constraint->point_count > 0) {
            if (write_index != read_index) {
                g_scene.cached_contact_constraints[write_index] = g_scene.cached_contact_constraints[read_index];
            }
            write_index++;
        }
    }
    g_scene.cached_contact_constraint_count = write_index;

    // Rebuild contact map
    hash_map_clear(&g_scene.contact_map);

    for (int i = 0; i < g_scene.cached_contact_constraint_count; i++) {
        contact_constraint* c = &g_scene.cached_contact_constraints[i];
        c->next_same_pid_index = -1;
        
        intptr_t existing_idx_plus_1 = (intptr_t)hash_map_get(&g_scene.contact_map, c->pid);
        
        if (existing_idx_plus_1 != 0) {
            c->next_same_pid_index = (int)existing_idx_plus_1 - 1;
        }
        hash_map_set(&g_scene.contact_map, c->pid, (void*)(intptr_t)(i + 1));
    }
}

/// @brief Detect all contacts (object-to-object and object-to-mesh)
static void collision_scene_detect_all_contacts() {
    // Refresh contacts (update world pos, mark inactive)
    collision_scene_refresh_contacts();

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
            
            // Optimization: Skip duplicate pairs
            // Since contact_pair_id is symmetric, we only need to check each pair once.
            // We enforce a < b by entity_id.
            // However, if b is sleeping, it won't perform the check, so we must do it regardless of ID order.
            if (!b->_is_sleeping && a->entity_id > b->entity_id) continue;

            // Narrow phase - only detect, don't resolve!
            detect_contact_object_to_object(a, b);
        }
    }
    #define MAX_SWEPT_ITERATIONS    6
    // Detect object-to-mesh collisions
    if (g_scene.mesh_collider) {
        for (int i = 0; i < g_scene.objectCount; i++)
        {
            physics_object* obj = g_scene.elements[i].object;

            // Skip if all position axes are frozen (object can't move anyway)
            bool all_position_frozen = (obj->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL;

            // Detect mesh collision for all non-sleeping, non-kinematic, tangible objects
            if (obj->_is_sleeping || obj->is_trigger || obj->is_kinematic || all_position_frozen || !(obj->collision_layers & COLLISION_LAYER_TANGIBLE)) {
                continue;
            }

            for (int i = 0; i < MAX_SWEPT_ITERATIONS; i += 1)
            {
                Vector3 displacement;
                vector3FromTo(&obj->_prev_step_pos, obj->position, &displacement);
                Vector3 bounding_box_size;
                vector3Sub(&obj->bounding_box.max, &obj->bounding_box.min, &bounding_box_size);
                vector3Scale(&bounding_box_size, &bounding_box_size, 0.5f);

                // if the object has moved more than the bounding box size perform a swept collision check with immediate response
                if (fabs(displacement.x) > bounding_box_size.x ||
                    fabs(displacement.y) > bounding_box_size.y ||
                    fabs(displacement.z) > bounding_box_size.z)
                {
                    if (!collide_object_to_mesh_swept(obj, g_scene.mesh_collider, &obj->_prev_step_pos))
                    {
                        break;
                    }
                }
                // otherwise just do a normal collision check
                else
                {
                    detect_contacts_object_to_mesh(obj, g_scene.mesh_collider);
                    break;
                }
            }
        
        }
    }

    // Remove contacts that were not detected this frame
    collision_scene_remove_inactive_contacts();
}

/// @brief Pre-solve: calculate effective masses and prepare constraint data
static void collision_scene_pre_solve_contacts() {
    for (int i = 0; i < g_scene.cached_contact_constraint_count; i++) {
        contact_constraint* cont_constraint = &g_scene.cached_contact_constraints[i];

        if (!cont_constraint->is_active || cont_constraint->is_trigger) continue;

        physics_object* a = cont_constraint->objectA;
        physics_object* b = cont_constraint->objectB;

        // Calculate center of mass for both objects (shared across all points)
        Vector3 centerOfMassA = gZeroVec;
        Vector3 centerOfMassB = gZeroVec;

        Vector3 normal = cont_constraint->normal;

        if (a) {
            centerOfMassA = a->_world_center_of_mass;
        }

        if (b) {
            centerOfMassB = b->_world_center_of_mass;
        }

        // Calculate tangent vectors for friction (shared across all points)
        vector3CalculateTangents(&cont_constraint->normal, &cont_constraint->tangent_u, &cont_constraint->tangent_v);

        // Process each contact point
        for (int p = 0; p < cont_constraint->point_count; p++) {
            contact_point* cont_point = &cont_constraint->points[p];

            // Calculate rA and rB (contact point relative to center of mass)
            if (a) {
                vector3Sub(&cont_point->contactA, &centerOfMassA, &cont_point->a_to_contact);
            } else {
                cont_point->a_to_contact = gZeroVec;
            }

            if (b) {
                vector3Sub(&cont_point->contactB, &centerOfMassB, &cont_point->b_to_contact);
            } else {
                cont_point->b_to_contact = gZeroVec;
            }

            // Calculate effective mass for normal direction
            bool aMovementConstrained = a && (a->is_kinematic || ((a->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));
            bool bMovementConstrained = b && (b->is_kinematic || ((b->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));

            float invMassA = 0.0f;
            float invMassB = 0.0f;

            if (a && !aMovementConstrained) {
                bool constrainedAlongNormal = ((a->constraints & CONSTRAINTS_FREEZE_POSITION_X) && fabsf(normal.x) > 0.01f) ||
                                              ((a->constraints & CONSTRAINTS_FREEZE_POSITION_Y) && fabsf(normal.y) > 0.01f) ||
                                              ((a->constraints & CONSTRAINTS_FREEZE_POSITION_Z) && fabsf(normal.z) > 0.01f);
                invMassA = constrainedAlongNormal ? 0.0f : a->_inv_mass;
            }

            if (b && !bMovementConstrained) {
                bool constrainedAlongNormal = ((b->constraints & CONSTRAINTS_FREEZE_POSITION_X) && fabsf(normal.x) > 0.01f) ||
                                              ((b->constraints & CONSTRAINTS_FREEZE_POSITION_Y) && fabsf(normal.y) > 0.01f) ||
                                              ((b->constraints & CONSTRAINTS_FREEZE_POSITION_Z) && fabsf(normal.z) > 0.01f);
                invMassB = constrainedAlongNormal ? 0.0f : b->_inv_mass;
            }

            float denominator = invMassA + invMassB;

            // Add rotational inertia term for A
            if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossN;
                vector3Cross(&cont_point->a_to_contact, &normal, &rCrossN);
                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(a, &rCrossN, &torquePerImpulse);
                denominator += vector3Dot(&rCrossN, &torquePerImpulse);
            }

            // Add rotational inertia term for B
            if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossN;
                vector3Cross(&cont_point->b_to_contact, &normal, &rCrossN);
                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(b, &rCrossN, &torquePerImpulse);
                denominator += vector3Dot(&rCrossN, &torquePerImpulse);
            }

            if (denominator < EPSILON) denominator = EPSILON;
            cont_point->normal_mass = 1.0f / denominator;

            // Calculate effective mass for tangent U
            float denominator_u = invMassA + invMassB;
            if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossT;
                vector3Cross(&cont_point->a_to_contact, &cont_constraint->tangent_u, &rCrossT);
                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(a, &rCrossT, &torquePerImpulse);
                denominator_u += vector3Dot(&rCrossT, &torquePerImpulse);
            }
            if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossT;
                vector3Cross(&cont_point->b_to_contact, &cont_constraint->tangent_u, &rCrossT);
                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(b, &rCrossT, &torquePerImpulse);
                denominator_u += vector3Dot(&rCrossT, &torquePerImpulse);
            }
            if (denominator_u < EPSILON) denominator_u = EPSILON;
            cont_point->tangent_mass_u = 1.0f / denominator_u;

            // Calculate effective mass for tangent V
            float denominator_v = invMassA + invMassB;
            if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossT;
                vector3Cross(&cont_point->a_to_contact, &cont_constraint->tangent_v, &rCrossT);
                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(a, &rCrossT, &torquePerImpulse);
                denominator_v += vector3Dot(&rCrossT, &torquePerImpulse);
            }
            if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossT;
                vector3Cross(&cont_point->b_to_contact, &cont_constraint->tangent_v, &rCrossT);
                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(b, &rCrossT, &torquePerImpulse);
                denominator_v += vector3Dot(&rCrossT, &torquePerImpulse);
            }
            if (denominator_v < EPSILON) denominator_v = EPSILON;
            cont_point->tangent_mass_v = 1.0f / denominator_v;

            // Calculate relative velocity for restitution
            Vector3 contactVelA = gZeroVec;
            Vector3 contactVelB = gZeroVec;

            if (a && !a->is_kinematic) {
                contactVelA = a->velocity;
                if (a->rotation) {
                    Vector3 angularContribution;
                    vector3Cross(&a->angular_velocity, &cont_point->a_to_contact, &angularContribution);
                    vector3Add(&contactVelA, &angularContribution, &contactVelA);
                }
            }

            if (b && !b->is_kinematic) {
                contactVelB = b->velocity;
                if (b->rotation) {
                    Vector3 angularContribution;
                    vector3Cross(&b->angular_velocity, &cont_point->b_to_contact, &angularContribution);
                    vector3Add(&contactVelB, &angularContribution, &contactVelB);
                }
            }

            Vector3 relVel;
            vector3Sub(&contactVelA, &contactVelB, &relVel);
            float normalVelocity = vector3Dot(&relVel, &cont_constraint->normal);

            // Calculate velocity bias (restitution)
            cont_point->velocity_bias = 0.0f;
            if (normalVelocity < -0.5f) { // Threshold for bouncing
                cont_point->velocity_bias = cont_constraint->combined_bounce * normalVelocity;
            }
        }
    }
}


/// @brief Warm start: apply accumulated impulses from previous frame
static void collision_scene_warm_start() {
    for (int i = 0; i < g_scene.cached_contact_constraint_count; i++) {
        contact_constraint* cc = &g_scene.cached_contact_constraints[i];

        if (!cc->is_active || cc->is_trigger) continue;

        physics_object* a = cc->objectA;
        physics_object* b = cc->objectB;

        // Process each contact point
        for (int p = 0; p < cc->point_count; p++)
        {
            contact_point *cp = &cc->points[p];
            // Apply accumulated normal impulse
            Vector3 impulse;
            vector3Scale(&cc->normal, &impulse, cp->accumulated_normal_impulse);

            // Apply to object A
            if (a && !a->is_kinematic)
            {
                Vector3 linearImpulse;
                vector3Scale(&impulse, &linearImpulse, a->_inv_mass);

                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    a->velocity.x += linearImpulse.x;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    a->velocity.y += linearImpulse.y;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    a->velocity.z += linearImpulse.z;

                if (a->rotation)
                {
                    Vector3 angularImpulse;
                    vector3Cross(&cp->a_to_contact, &impulse, &angularImpulse);
                    physics_object_apply_angular_impulse(a, &angularImpulse);
                }
            }

            // Apply to object B (opposite direction)
            if (b && !b->is_kinematic)
            {
                Vector3 linearImpulse;
                vector3Scale(&impulse, &linearImpulse, -b->_inv_mass);

                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    b->velocity.x += linearImpulse.x;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    b->velocity.y += linearImpulse.y;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    b->velocity.z += linearImpulse.z;

                if (b->rotation)
                {
                    Vector3 angularImpulse;
                    vector3Cross(&cp->b_to_contact, &impulse, &angularImpulse);
                    vector3Negate(&angularImpulse, &angularImpulse);
                    physics_object_apply_angular_impulse(b, &angularImpulse);
                }
            }

            // Apply accumulated tangent impulses for friction
            Vector3 tangentImpulseU;
            vector3Scale(&cc->tangent_u, &tangentImpulseU, cp->accumulated_tangent_impulse_u);

            Vector3 tangentImpulseV;
            vector3Scale(&cc->tangent_v, &tangentImpulseV, cp->accumulated_tangent_impulse_v);

            // Apply tangent_u to A
            if (a && !a->is_kinematic)
            {
                Vector3 linearTangentU;
                vector3Scale(&tangentImpulseU, &linearTangentU, a->_inv_mass);
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    a->velocity.x += linearTangentU.x;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    a->velocity.y += linearTangentU.y;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    a->velocity.z += linearTangentU.z;

                if (a->rotation)
                {
                    Vector3 angularTangentU;
                    vector3Cross(&cp->a_to_contact, &tangentImpulseU, &angularTangentU);
                    physics_object_apply_angular_impulse(a, &angularTangentU);
                }
            }

            // Apply tangent_u to B (opposite)
            if (b && !b->is_kinematic)
            {
                Vector3 linearTangentU;
                vector3Scale(&tangentImpulseU, &linearTangentU, -b->_inv_mass);
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    b->velocity.x += linearTangentU.x;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    b->velocity.y += linearTangentU.y;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    b->velocity.z += linearTangentU.z;

                if (b->rotation)
                {
                    Vector3 angularTangentU;
                    vector3Cross(&cp->b_to_contact, &tangentImpulseU, &angularTangentU);
                    vector3Negate(&angularTangentU, &angularTangentU);
                    physics_object_apply_angular_impulse(b, &angularTangentU);
                }
            }

            // Apply tangent_v to A
            if (a && !a->is_kinematic)
            {
                Vector3 linearTangentV;
                vector3Scale(&tangentImpulseV, &linearTangentV, a->_inv_mass);
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    a->velocity.x += linearTangentV.x;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    a->velocity.y += linearTangentV.y;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    a->velocity.z += linearTangentV.z;

                if (a->rotation)
                {
                    Vector3 angularTangentV;
                    vector3Cross(&cp->a_to_contact, &tangentImpulseV, &angularTangentV);
                    physics_object_apply_angular_impulse(a, &angularTangentV);
                }
            }

            // Apply tangent_v to B (opposite)
            if (b && !b->is_kinematic)
            {
                Vector3 linearTangentV;
                vector3Scale(&tangentImpulseV, &linearTangentV, -b->_inv_mass);
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    b->velocity.x += linearTangentV.x;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    b->velocity.y += linearTangentV.y;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    b->velocity.z += linearTangentV.z;

                if (b->rotation)
                {
                    Vector3 angularTangentV;
                    vector3Cross(&cp->b_to_contact, &tangentImpulseV, &angularTangentV);
                    vector3Negate(&angularTangentV, &angularTangentV);
                    physics_object_apply_angular_impulse(b, &angularTangentV);
                }
            }
        }
    }
}

/// @brief Solve velocity constraints iteratively
static void collision_scene_solve_velocity_constraints()
{
    for (int i = 0; i < g_scene.cached_contact_constraint_count; i++)
    {
        contact_constraint *cc = &g_scene.cached_contact_constraints[i];

        if (!cc->is_active || cc->is_trigger)
            continue;

        physics_object *a = cc->objectA;
        physics_object *b = cc->objectB;

        // Process each contact point
        for (int p = 0; p < cc->point_count; p++)
        {
            contact_point *cp = &cc->points[p];

            // Calculate contact velocities
            Vector3 contactVelA = gZeroVec;
            Vector3 contactVelB = gZeroVec;

            if (a && !a->is_kinematic)
            {
                contactVelA = a->velocity;
                if (a->rotation)
                {
                    Vector3 angularContribution;
                    vector3Cross(&a->angular_velocity, &cp->a_to_contact, &angularContribution);
                    vector3Add(&contactVelA, &angularContribution, &contactVelA);
                }
            }

            if (b && !b->is_kinematic)
            {
                contactVelB = b->velocity;
                if (b->rotation)
                {
                    Vector3 angularContribution;
                    vector3Cross(&b->angular_velocity, &cp->b_to_contact, &angularContribution);
                    vector3Add(&contactVelB, &angularContribution, &contactVelB);
                }
            }

            // Calculate relative velocity
            Vector3 relVel;
            vector3Sub(&contactVelA, &contactVelB, &relVel);
            float normalVelocity = vector3Dot(&relVel, &cc->normal);

            // Calculate lambda (impulse change)
            // Use pre-calculated velocity bias (restitution)
            float lambda = -(normalVelocity + cp->velocity_bias) * cp->normal_mass;

            // Clamp accumulated impulse (key difference from old approach!)
            float oldImpulse = cp->accumulated_normal_impulse;
            cp->accumulated_normal_impulse = maxf(oldImpulse + lambda, 0.0f);
            lambda = cp->accumulated_normal_impulse - oldImpulse;

            if (fabsf(lambda) < EPSILON)
                continue;

            // Apply lambda (the change, not total)
            Vector3 impulse;
            vector3Scale(&cc->normal, &impulse, lambda);

            // Apply to object A
            if (a && !a->is_kinematic)
            {
                Vector3 linearImpulse;
                vector3Scale(&impulse, &linearImpulse, a->_inv_mass);

                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    a->velocity.x += linearImpulse.x;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    a->velocity.y += linearImpulse.y;
                if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    a->velocity.z += linearImpulse.z;

                if (a->rotation)
                {
                    Vector3 angularImpulse;
                    vector3Cross(&cp->a_to_contact, &impulse, &angularImpulse);
                    
                    Vector3 deltaOmega;
                    physics_object_apply_world_inertia(a, &angularImpulse, &deltaOmega);
                    vector3Add(&a->angular_velocity, &deltaOmega, &a->angular_velocity);
                }
            }

            // Apply to object B
            if (b && !b->is_kinematic)
            {
                Vector3 linearImpulse;
                vector3Scale(&impulse, &linearImpulse, -b->_inv_mass);

                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                    b->velocity.x += linearImpulse.x;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                    b->velocity.y += linearImpulse.y;
                if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                    b->velocity.z += linearImpulse.z;

                if (b->rotation)
                {
                    Vector3 angularImpulse;
                    vector3Cross(&cp->b_to_contact, &impulse, &angularImpulse);
                    vector3Negate(&angularImpulse, &angularImpulse);
                    
                    Vector3 deltaOmega;
                    physics_object_apply_world_inertia(b, &angularImpulse, &deltaOmega);
                    vector3Add(&b->angular_velocity, &deltaOmega, &b->angular_velocity);
                }
            }
#ifndef DEBUG_IGNORE_FRICTION
            // Handle friction with proper accumulation
            if (cc->combined_friction > 0.0f)
            {
                // Recalculate relative velocity after normal impulse
                if (a && !a->is_kinematic)
                {
                    contactVelA = a->velocity;
                    if (a->rotation)
                    {
                        Vector3 angularContribution;
                        vector3Cross(&a->angular_velocity, &cp->a_to_contact, &angularContribution);
                        vector3Add(&contactVelA, &angularContribution, &contactVelA);
                    }
                }

                if (b && !b->is_kinematic)
                {
                    contactVelB = b->velocity;
                    if (b->rotation)
                    {
                        Vector3 angularContribution;
                        vector3Cross(&b->angular_velocity, &cp->b_to_contact, &angularContribution);
                        vector3Add(&contactVelB, &angularContribution, &contactVelB);
                    }
                }

                vector3Sub(&contactVelA, &contactVelB, &relVel);

                // Calculate tangential velocity components along tangent_u and tangent_v
                float vTangentU = vector3Dot(&relVel, &cc->tangent_u);
                float vTangentV = vector3Dot(&relVel, &cc->tangent_v);

                // Calculate friction impulse changes (lambda) for both tangent directions
                float lambdaU = -vTangentU * cp->tangent_mass_u;
                float lambdaV = -vTangentV * cp->tangent_mass_v;

                // Calculate new accumulated tangent impulses
                float newAccumU = cp->accumulated_tangent_impulse_u + lambdaU;
                float newAccumV = cp->accumulated_tangent_impulse_v + lambdaV;

                // Clamp to friction cone (Coulomb's law: |tangent_impulse| <= friction * normal_impulse)
                float maxFriction = cc->combined_friction * cp->accumulated_normal_impulse;
                float tangentMagnitude = sqrtf(newAccumU * newAccumU + newAccumV * newAccumV);

                if (tangentMagnitude > maxFriction)
                {
                    float scale = maxFriction / tangentMagnitude;
                    newAccumU *= scale;
                    newAccumV *= scale;
                }

                // Calculate actual impulse deltas to apply
                lambdaU = newAccumU - cp->accumulated_tangent_impulse_u;
                lambdaV = newAccumV - cp->accumulated_tangent_impulse_v;

                // Update accumulated values
                cp->accumulated_tangent_impulse_u = newAccumU;
                cp->accumulated_tangent_impulse_v = newAccumV;

                // Apply tangent_u impulse
                if (fabsf(lambdaU) > EPSILON)
                {
                    Vector3 tangentImpulseU;
                    vector3Scale(&cc->tangent_u, &tangentImpulseU, lambdaU);

                    if (a && !a->is_kinematic)
                    {
                        Vector3 linearImpulse;
                        vector3Scale(&tangentImpulseU, &linearImpulse, a->_inv_mass);
                        if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                            a->velocity.x += linearImpulse.x;
                        if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                            a->velocity.y += linearImpulse.y;
                        if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                            a->velocity.z += linearImpulse.z;

                        if (a->rotation)
                        {
                            Vector3 angularImpulse;
                            vector3Cross(&cp->a_to_contact, &tangentImpulseU, &angularImpulse);
                            physics_object_apply_angular_impulse(a, &angularImpulse);
                        }
                    }

                    if (b && !b->is_kinematic)
                    {
                        Vector3 linearImpulse;
                        vector3Scale(&tangentImpulseU, &linearImpulse, -b->_inv_mass);
                        if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                            b->velocity.x += linearImpulse.x;
                        if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                            b->velocity.y += linearImpulse.y;
                        if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                            b->velocity.z += linearImpulse.z;

                        if (b->rotation)
                        {
                            Vector3 angularImpulse;
                            vector3Cross(&cp->b_to_contact, &tangentImpulseU, &angularImpulse);
                            vector3Negate(&angularImpulse, &angularImpulse);
                            physics_object_apply_angular_impulse(b, &angularImpulse);
                        }
                    }
                }

                // Apply tangent_v impulse
                if (fabsf(lambdaV) > EPSILON)
                {
                    Vector3 tangentImpulseV;
                    vector3Scale(&cc->tangent_v, &tangentImpulseV, lambdaV);

                    if (a && !a->is_kinematic)
                    {
                        Vector3 linearImpulse;
                        vector3Scale(&tangentImpulseV, &linearImpulse, a->_inv_mass);
                        if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                            a->velocity.x += linearImpulse.x;
                        if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                            a->velocity.y += linearImpulse.y;
                        if (!(a->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                            a->velocity.z += linearImpulse.z;

                        if (a->rotation)
                        {
                            Vector3 angularImpulse;
                            vector3Cross(&cp->a_to_contact, &tangentImpulseV, &angularImpulse);
                            physics_object_apply_angular_impulse(a, &angularImpulse);
                        }
                    }

                    if (b && !b->is_kinematic)
                    {
                        Vector3 linearImpulse;
                        vector3Scale(&tangentImpulseV, &linearImpulse, -b->_inv_mass);
                        if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_X))
                            b->velocity.x += linearImpulse.x;
                        if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Y))
                            b->velocity.y += linearImpulse.y;
                        if (!(b->constraints & CONSTRAINTS_FREEZE_POSITION_Z))
                            b->velocity.z += linearImpulse.z;

                        if (b->rotation)
                        {
                            Vector3 angularImpulse;
                            vector3Cross(&cp->b_to_contact, &tangentImpulseV, &angularImpulse);
                            vector3Negate(&angularImpulse, &angularImpulse);
                            physics_object_apply_angular_impulse(b, &angularImpulse);
                        }
                    }
                }
            }
#endif
        }
    }
}

/// @brief Solve position constraints iteratively
static void collision_scene_solve_position_constraints() {
    const float slop = 0.01f;
    const float steeringConstant = 0.2f;
    const float maxCorrection = 0.08f;

    for (int i = 0; i < g_scene.cached_contact_constraint_count; i++) {
        contact_constraint* cc = &g_scene.cached_contact_constraints[i];

        if (!cc->is_active || cc->is_trigger) continue;
        

        physics_object* a = cc->objectA;
        physics_object* b = cc->objectB;

        // Process each contact point
        for (int p = 0; p < cc->point_count; p++)
        {
            contact_point *cp = &cc->points[p];
            if (cp->penetration < slop)
                continue;

            const float steeringForce = clampf(steeringConstant * (cp->penetration + slop), 0, maxCorrection);

            bool aMovementConstrained = a && (a->is_kinematic || ((a->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));
            bool bMovementConstrained = b && (b->is_kinematic || ((b->constraints & CONSTRAINTS_FREEZE_POSITION_ALL) == CONSTRAINTS_FREEZE_POSITION_ALL));

            float invMassA = 0.0f;
            float invMassB = 0.0f;

            Vector3 effectiveNormalA = cc->normal;
            Vector3 effectiveNormalB = cc->normal;

            float normal_dot_inv = 1.0f / vector3Dot(&cc->normal, &cc->normal);

            if (a && !aMovementConstrained)
            {
                if (a->constraints & CONSTRAINTS_FREEZE_POSITION_X)
                    effectiveNormalA.x = 0.0f;
                if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Y)
                    effectiveNormalA.y = 0.0f;
                if (a->constraints & CONSTRAINTS_FREEZE_POSITION_Z)
                    effectiveNormalA.z = 0.0f;

                float normalDotA = vector3Dot(&effectiveNormalA, &cc->normal);
                invMassA = a->_inv_mass * (normalDotA * normalDotA) * normal_dot_inv;
            }

            if (b && !bMovementConstrained)
            {
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

            // Add rotational inertia term for A
            if (a && a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossN;
                vector3Cross(&cp->a_to_contact, &cc->normal, &rCrossN);

                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(a, &rCrossN, &torquePerImpulse);

                invMassSum += vector3Dot(&rCrossN, &torquePerImpulse);
            }

            // Add rotational inertia term for B
            if (b && b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                Vector3 rCrossN;
                vector3Cross(&cp->b_to_contact, &cc->normal, &rCrossN);

                Vector3 torquePerImpulse;
                physics_object_apply_world_inertia(b, &rCrossN, &torquePerImpulse);

                invMassSum += vector3Dot(&rCrossN, &torquePerImpulse);
            }

            if (invMassSum == 0.0f)
                continue;

            float correctionMag = steeringForce / invMassSum;
            Vector3 impulse;
            vector3Scale(&cc->normal, &impulse, correctionMag);

            // Apply correction
            if (a && !aMovementConstrained)
            {
                if (invMassA > 0.0f) {
                    vector3AddScaled(a->position, &effectiveNormalA, correctionMag * invMassA, a->position);
                }
                
                if (a->rotation && !((a->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                    Vector3 angularImpulse;
                    vector3Cross(&cp->a_to_contact, &impulse, &angularImpulse);
                    physics_object_apply_angular_impulse_to_rotation(a, &angularImpulse);
                }
            }

            if (b && !bMovementConstrained)
            {
                if (invMassB > 0.0f) {
                    vector3AddScaled(b->position, &effectiveNormalB, -correctionMag * invMassB, b->position);
                }

                if (b->rotation && !((b->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL)) {
                    Vector3 angularImpulse;
                    vector3Cross(&cp->b_to_contact, &impulse, &angularImpulse);
                    vector3Negate(&angularImpulse, &angularImpulse);
                    physics_object_apply_angular_impulse_to_rotation(b, &angularImpulse);
                }
            }
        }
    }
}

/// @brief performs a physics step on all objects in the scene using iterative constraint solver
void collision_scene_step() {
    struct collision_scene_element* element;

    // ========================================================================
    // PHASE 0: Update world inertia tensors
    // ========================================================================
    for (int i = 0; i < g_scene.objectCount; i++) {
        physics_object* obj = g_scene.elements[i].object;
        if (!obj->_is_sleeping) {
            physics_object_update_world_inertia(obj);
        }
    }

    // ========================================================================
    // PHASE 1: Apply gravity and integrate velocities
    // ========================================================================
    for (int i = 0; i < g_scene.objectCount; i++) {
        element = &g_scene.elements[i];
        physics_object* obj = element->object;

        if (!obj->_is_sleeping && obj->has_gravity && !obj->is_kinematic)
        {
            obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar;
        }

        // Release old contacts from previous frame
        collision_scene_release_object_contacts(obj);

        // Integrate acceleration into velocity (but don't update position yet!)
        physics_object_integrate_velocity(obj);

        // Update angular velocity
        physics_object_integrate_angular_velocity(obj);
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
    for (int iter = 0; iter < VELOCITY_CONSTRAINT_SOLVER_ITERATIONS; iter++) {
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

        // Integrate angular velocity into rotation
        physics_object_integrate_rotation(obj);

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
    // PHASE 7: Solve position constraints iteratively
    // ========================================================================
    for (int iter = 0; iter < POSITION_CONSTRAINT_SOLVER_ITERATIONS; iter++) {
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
                physics_object_sleep(obj);
            }
        }
        else
        {
            physics_object_wake(obj);
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