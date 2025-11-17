#include "collision_scene.h"

#include <malloc.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <libdragon.h>

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

    // First loop: Position updates and AABB maintenance
    for (int i = 0; i < g_scene.objectCount; i++) {
        element = &g_scene.elements[i];
        physics_object* obj = element->object;
        obj->_ground_support_factor = 0;

        if (!obj->_is_sleeping && obj->has_gravity && !obj->is_kinematic)
        {
            // Check if object has ground contact
            float ground_support_factor = 0.0f;
            if (obj->active_contacts)
            {
                contact *c = obj->active_contacts;
                while (c)
                {
                    // Determine the contact normal pointing most up
                    if (c->normal.y > ground_support_factor)
                    {
                        ground_support_factor = c->normal.y;
                        break;
                    }
                    c = c->next;
                }
            }
            obj->_ground_support_factor = ground_support_factor;
            // scale the applied gravity down according to how much the object is supported from underneath
            ground_support_factor = clampf(ground_support_factor, 0.0f, 0.8f);
            obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar * (1.0f - ground_support_factor);
        }

        collision_scene_release_object_contacts(obj);
        physics_object_update_velocity_semi_implicit_euler(obj);
        physics_object_update_angular_velocity(obj);

        // Track movement for AABB updates
        const bool has_moved = !vector3IsIdentical(&obj->_prev_step_pos, obj->position);
        const bool has_rotated = obj->rotation ? !quatIsIdentical(obj->rotation, &obj->_prev_step_rot) : false;
        g_scene._moved_flags[i] = has_moved;
        g_scene._rotated_flags[i] = has_rotated;

        if (!obj->_is_sleeping && (has_moved || has_rotated)) {
            collision_scene_collide_phys_object(element);

            physics_object_recalculate_aabb(obj);
            Vector3 displacement;
            vector3Sub(obj->position, &obj->_prev_step_pos, &displacement);
            AABB_tree_move_node(&g_scene.object_aabbtree, obj->_aabb_tree_node_id,
                            obj->bounding_box, &displacement);
        }
    }
    g_scene._sleepy_count = 0;
    // Second loop: Mesh Collision, Constraints and Sleep State Update
    for (int i = 0; i < g_scene.objectCount; i++) {
        element = &g_scene.elements[i];
        physics_object* obj = element->object;

        // Only do mesh collision if the object is not sleeping, not fixed in place, is tangible and has moved or rotated previously 
        if (g_scene.mesh_collider && !obj->_is_sleeping && !obj->is_kinematic && 
            (obj->collision_layers & (COLLISION_LAYER_TANGIBLE)) && (g_scene._moved_flags[i] || g_scene._rotated_flags[i])) {
            collision_scene_collide_object_to_static(obj, &obj->_prev_step_pos);
        }

        // Apply physical constraints to the object
        physics_object_apply_position_constraints(obj);

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
                //get rid of any residual velocity so object is actually at rest
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
            //only update the previous position if the object is awake so that even small drift would accumulate and be registered eventually
            obj->_prev_step_pos = *obj->position;
            if (obj->rotation)
            {
                obj->_prev_step_rot = *obj->rotation;
            }
        }
        else{
            //TODO: for debugging - remove
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