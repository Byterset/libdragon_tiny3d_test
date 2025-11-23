#ifndef __COLLISION_COLLISION_SCENE_H__
#define __COLLISION_COLLISION_SCENE_H__

#include "physics_object.h"
#include "../collision/mesh_collider.h"
#include "../util/hash_map.h"
#include "../collision/aabb_tree.h"
#include "contact.h"


#define MAX_PHYSICS_OBJECTS 64
#define MAX_CACHED_CONTACTS 512

struct collision_scene_element {
    physics_object* object;
};

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

void collision_scene_reset();
struct collision_scene* collision_scene_get();
void collision_scene_add(physics_object* object);
void collision_scene_remove(physics_object* object);

physics_object* collision_scene_find_object(entity_id id);

void collision_scene_use_static_collision(struct mesh_collider* mesh_collider);
void collision_scene_remove_static_collision();

void collision_scene_step();

contact* collision_scene_new_contact();

static inline void apply_world_inertia(physics_object* obj, Vector3* in, Vector3* out) {
    float* m = obj->_inv_world_inertia_tensor;
    out->x = m[0]*in->x + m[1]*in->y + m[2]*in->z;
    out->y = m[3]*in->x + m[4]*in->y + m[5]*in->z;
    out->z = m[6]*in->x + m[7]*in->y + m[8]*in->z;
}

#endif