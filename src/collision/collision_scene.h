#ifndef __COLLISION_COLLISION_SCENE_H__
#define __COLLISION_COLLISION_SCENE_H__

#include "physics_object.h"
#include "../collision/mesh_collider.h"
#include "../util/hash_map.h"
#include "../collision/aabb_tree.h"
#include "contact.h"


#define MAX_PHYSICS_OBJECTS 64
#define MAX_ACTIVE_CONTACTS 128
#define MAX_CACHED_CONTACTS 256

#define VELOCITY_CONSTRAINT_SOLVER_ITERATIONS 5
#define POSITION_CONSTRAINT_SOLVER_ITERATIONS 4


/// @brief A wrapper for a physics object in the collision scene
struct collision_scene_element {
    physics_object* object;
};


/// @brief The main collision scene structure holding all physics objects and contacts
struct collision_scene {
    struct collision_scene_element* elements;
    contact* next_free_contact;
    contact* all_contacts;
    struct hash_map entity_mapping;
    uint16_t objectCount;
    uint16_t capacity;
    AABB_tree object_aabbtree;
    struct mesh_collider* mesh_collider;
    bool _moved_flags[MAX_PHYSICS_OBJECTS];
    bool _rotated_flags[MAX_PHYSICS_OBJECTS];
    uint16_t _sleepy_count;

    // Iterative constraint solver data
    contact_constraint* cached_contact_constraints;
    int cached_contact_constraint_count;
    struct hash_map contact_map;
};


/// @brief Resets the collision scene, clearing all objects and contacts
void collision_scene_reset();

/// @brief Returns the global collision scene instance
struct collision_scene* collision_scene_get_instance();


/// @brief Adds a physics object to the collision scene
/// @param object The object to add
void collision_scene_add(physics_object* object);


/// @brief Removes a physics object from the collision scene
/// @param object The object to remove
void collision_scene_remove(physics_object* object);


/// @brief Finds a physics object in the scene by its entity ID
/// @param id The entity ID to search for
/// @return The physics object if found, NULL otherwise
physics_object* collision_scene_find_object(entity_id id);


/// @brief Sets the static mesh collider for the scene
/// @param mesh_collider The mesh collider to use
void collision_scene_use_static_collision(struct mesh_collider* mesh_collider);


/// @brief Removes the current static collision mesh from the scene
void collision_scene_remove_static_collision();


/// @brief Performs a physics step on all objects in the scene
void collision_scene_step();


/// @brief Allocates a new contact from the scene's pool
/// @return A pointer to the new contact, or NULL if the pool is empty
contact* collision_scene_allocate_contact();

#endif