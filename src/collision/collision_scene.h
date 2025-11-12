#ifndef __COLLISION_COLLISION_SCENE_H__
#define __COLLISION_COLLISION_SCENE_H__

#include "physics_object.h"
#include "../collision/mesh_collider.h"
#include "../util/hash_map.h"
#include "../collision/aabbtree.h"
#include "contact.h"

typedef int collision_id;

#define MAX_PHYSICS_OBJECTS 64

struct collision_scene_element {
    struct physics_object* object;
};

struct collision_scene {
    struct collision_scene_element* elements;
    struct contact* next_free_contact;
    struct contact* all_contacts;
    struct hash_map entity_mapping;
    uint16_t objectCount;
    uint16_t capacity;
    AABBTree object_aabbtree;
    struct mesh_collider* mesh_collider;
    bool _moved_flags[MAX_PHYSICS_OBJECTS];
    bool _rotated_flags[MAX_PHYSICS_OBJECTS];
    uint16_t _sleepy_count;
};

void collision_scene_reset();
struct collision_scene* collision_scene_get();
void collision_scene_add(struct physics_object* object);
void collision_scene_remove(struct physics_object* object);

struct physics_object* collision_scene_find_object(entity_id id);

void collision_scene_use_static_collision(struct mesh_collider* mesh_collider);
void collision_scene_remove_static_collision();

void collision_scene_step();

struct contact* collision_scene_new_contact();

#endif