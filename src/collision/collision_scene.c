#include "collision_scene.h"

#include <malloc.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <libdragon.h>

#include "mesh_collider.h"
#include "collide.h"
#include "collide_swept.h"
#include "../util/hash_map.h"

#include "../render/defs.h"
#include "../game/gamestate.h"

struct collision_scene g_scene;

void collision_scene_reset() {
    free(g_scene.elements);
    free(g_scene.all_contacts);
    AABBTree_free(&g_scene.object_aabbtree);
    hash_map_destroy(&g_scene.entity_mapping);

    hash_map_init(&g_scene.entity_mapping, MAX_PHYSICS_OBJECTS);
    AABBTree_create(&g_scene.object_aabbtree, MAX_PHYSICS_OBJECTS);
    g_scene.elements = malloc(sizeof(struct collision_scene_element) * MAX_PHYSICS_OBJECTS);
    g_scene.capacity = MAX_PHYSICS_OBJECTS;
    g_scene.objectCount = 0;
    AABBTree_free(&g_scene.mesh_collider->aabbtree);
    g_scene.mesh_collider = NULL;
    g_scene.all_contacts = malloc(sizeof(struct contact) * MAX_ACTIVE_CONTACTS);
    g_scene.contact_allocator_head = 0;
}

struct collision_scene* collision_scene_get() {
    return &g_scene;
}

void collision_scene_add(struct physics_object* object) {
    if (g_scene.objectCount >= g_scene.capacity) {
        g_scene.capacity *= 2;
        g_scene.elements = realloc(g_scene.elements, sizeof(struct collision_scene_element) * g_scene.capacity);
    }

    struct collision_scene_element* next = &g_scene.elements[g_scene.objectCount];

    next->object = object;

    g_scene.objectCount += 1;

    hash_map_set(&g_scene.entity_mapping, object->entity_id, object);
    object->_aabb_tree_node_id = AABBTreeNode_createNode(&g_scene.object_aabbtree, object->bounding_box, object);
}

/// @brief Returns the physics object associated with the given entity id if it exists in the collision scene.
struct physics_object* collision_scene_find_object(entity_id id) {
    if (!id) {
        return 0;
    }

    return hash_map_get(&g_scene.entity_mapping, id);
}

/**
 * @brief Clears the active contacts of a physics object.
 *
 * With the new linear allocator, contacts are released all at once at the start of the physics
 * step via collision_scene_reset_contacts(). This function just clears the object's contact list.
 *
 * @param object Pointer to the physics object whose active contacts are to be cleared.
 */
void collision_scene_release_object_contacts(struct physics_object* object) {
    // Simply clear the contact list - actual memory is reused via linear allocator reset
    object->active_contacts = NULL;
}

/// @brief Removes a physics object from the collision scene. 
///
/// The removed object will no longer be updated in the phys loop or considered for collision.
/// All contacts associated with the object will be released.
/// @param object 
void collision_scene_remove(struct physics_object* object) {
    bool has_found = false;

    for (int i = 0; i < g_scene.objectCount; ++i) {
        if (object == g_scene.elements[i].object) {
            collision_scene_release_object_contacts(object);
            has_found = true;
        }

        if (has_found) {
            g_scene.elements[i] = g_scene.elements[i + 1];
        }
    }

    if (has_found) {
        g_scene.objectCount -= 1;
    }
    AABBTree_removeLeaf(&g_scene.object_aabbtree, object->_aabb_tree_node_id, true);
    hash_map_delete(&g_scene.entity_mapping, object->entity_id);
}


void collision_scene_use_static_collision(struct mesh_collider* mesh_collider) {
    g_scene.mesh_collider = mesh_collider;
}


/// @brief Removes the current static collision mesh from the scene.
void collision_scene_remove_static_collision() {
    AABBTree_free(&g_scene.mesh_collider->aabbtree);
    g_scene.mesh_collider = NULL;
}


/// @brief Performs collision detection for a single physics object agains other objects in the scene.
///
/// First a broad phase detection using a AABB BVH tree is performed to find potential collision candidates.
/// Then a narrow phase detection (GJK/EPA) is performed to check for and resolve actual collisions.
/// @param element the scene element (phys object) to check for collisions
void collision_scene_collide_phys_object(struct collision_scene_element* element) {

    //perform a broad phase check to find potential collision candidates by traversing the AABB BVH tree 
    //and collecting leaf nodes that overlap with the object's bounding box
    int result_count = 0;
    int max_results = 10;
    NodeProxy results[max_results];
    AABBTree_queryBounds(&g_scene.object_aabbtree, &element->object->bounding_box, results, &result_count, max_results);

    //iterate over the list of candidates and perform detailed collision detection
    for (size_t i = 0; i < result_count; i++)
    {
        struct physics_object *other = (struct physics_object *)AABBTreeNode_getData(&g_scene.object_aabbtree, results[i]);
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

void collision_scene_collide_object_to_static(struct physics_object* object, Vector3* prev_pos) {

    for (int i = 0; i < MAX_SWEPT_ITERATIONS; i += 1)
    {
        Vector3 offset;
        vector3Sub(object->position, prev_pos, &offset);
        Vector3 bounding_box_size;
        vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &bounding_box_size);
        vector3Scale(&bounding_box_size, &bounding_box_size, 0.5f);

        // if the object has moved more than the bounding box size perform a swept collision check
        if (fabs(offset.x) > bounding_box_size.x ||
            fabs(offset.y) > bounding_box_size.y ||
            fabs(offset.z) > bounding_box_size.z)
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

/// @brief performs a physics step on all objects in the scene, updating their positions, velocities, Bounding Boxes and performing collision detection
void collision_scene_step() {
    struct collision_scene_element* element;

    // Pre-pass: Check ground contacts from PREVIOUS frame before clearing
    // Store results in a temporary array to avoid use-after-free
    bool hasGroundContact[MAX_PHYSICS_OBJECTS] = {false};
    for (int i = 0; i < g_scene.objectCount; ++i) {
        struct physics_object* obj = g_scene.elements[i].object;
        if (!obj->_is_sleeping && obj->has_gravity && !obj->is_kinematic && obj->active_contacts) {
            struct contact* c = obj->active_contacts;
            while (c) {
                // If contact normal points upward (more lenient threshold)
                if (c->normal.y > 0.5f) {
                    hasGroundContact[i] = true;
                    break;
                }
                c = c->next;
            }
        }
    }

    // Clear all contacts from previous frame
    for (int i = 0; i < g_scene.objectCount; ++i) {
        collision_scene_release_object_contacts(g_scene.elements[i].object);
    }

    // Reset allocator - safe now since all contacts are cleared
    g_scene.contact_allocator_head = 0;

    // First loop: Position updates and AABB maintenance
    for (int i = 0; i < g_scene.objectCount; ++i) {
        element = &g_scene.elements[i];
        struct physics_object* obj = element->object;

        if (!obj->_is_sleeping) {
            if (obj->has_gravity && !obj->is_kinematic) {
                // Use ground contact result from pre-pass
                if (!hasGroundContact[i]) {
                    obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar;
                } else {
                    // Apply minimal gravity to grounded objects (maintains contact pressure)
                    obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar * 0.10f;
                }
            }
        }
        physics_object_update_velocity_verlet(obj);
        physics_object_update_angular_velocity(obj);

        // Track movement for AABB updates
        const bool has_moved = !vector3Equals(&obj->_prev_step_pos, obj->position);
        const bool has_rotated = obj->rotation ? !quatIsIdentical(obj->rotation, &obj->_prev_step_rot) : false;
        g_scene.moved_flags[i] = has_moved;
        g_scene.rotated_flags[i] = has_rotated;

        if (!obj->_is_sleeping && (has_moved || has_rotated)) {
            // would be technically correct to do this after all objects have been updated but this saves a loop
            collision_scene_collide_phys_object(element);


            physics_object_recalculate_aabb(obj);
            Vector3 displacement;
            vector3Sub(obj->position, &obj->_prev_step_pos, &displacement);
            AABBTree_moveNode(&g_scene.object_aabbtree, obj->_aabb_tree_node_id,
                            obj->bounding_box, &displacement);
        }
    }

    // Second loop: Mesh Collision, Constraints and Sleep State Update
    for (int i = 0; i < g_scene.objectCount; ++i) {
        element = &g_scene.elements[i];
        struct physics_object* obj = element->object;

        // Only do mesh collision if the object is not sleeping, not fixed in place, is tangible and has moved or rotated previously 
        if (g_scene.mesh_collider && !obj->_is_sleeping && !obj->is_kinematic && 
            (obj->collision_layers & (COLLISION_LAYER_TANGIBLE)) && (g_scene.moved_flags[i] || g_scene.rotated_flags[i])) {
            collision_scene_collide_object_to_static(obj, &obj->_prev_step_pos);
        }

        // Apply physical constraints to the object
        physics_object_apply_constraints(obj);

        // Update sleep state

        // Check for external position changes (non-physics movement)
        const bool position_changed = vector3DistSqrd(obj->position, &obj->_prev_step_pos) > PHYS_OBJECT_POS_CHANGE_SLEEP_THRESHOLD_SQ;

        // Check for external rotation changes (non-physics rotation)
        // Use quaternion dot product - values close to ±1.0 indicate similar rotations
        bool rotation_changed = false;
        if (obj->rotation)
        {
            float quat_similarity = fabsf(quatDot(obj->rotation, &obj->_prev_step_rot));
            rotation_changed = quat_similarity < PHYS_OBJECT_ROT_SIMILARITY_SLEEP_THRESHOLD; // Allow tiny rotational drift
        }

        // Check physics-driven motion via velocities
        const bool has_linear_velocity = vector3MagSqrd(&obj->velocity) > PHYS_OBJECT_VELOCITY_SLEEP_THRESHOLD_SQ;
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
            }
        }
        else
        {
            // Update previous state with current state
            obj->_prev_step_pos = *obj->position;
            if (obj->rotation)
            {
                obj->_prev_step_rot = *obj->rotation;
            }
            obj->_is_sleeping = false;
            obj->_sleep_counter = 0;
        }
    }
}

/// @brief Returns a new contact from the global scene's contact pool, NULL if pool is exhausted.
struct contact* collision_scene_new_contact() {
    // Check if we've exhausted the contact pool
    if (g_scene.contact_allocator_head >= MAX_ACTIVE_CONTACTS) {
        return NULL;
    }

    // Return next available contact and increment head
    return &g_scene.all_contacts[g_scene.contact_allocator_head++];
}