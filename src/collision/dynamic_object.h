#ifndef __COLLISION_DYNAMIC_OBJECT_H__
#define __COLLISION_DYNAMIC_OBJECT_H__

#include "../entity/entity_id.h"
#include "../math/vector3.h"
#include "../math/vector2.h"
#include "../math/aabb.h"
#include "../math/quaternion.h"
#include "../collision/aabbtree.h"
#include "contact.h"
#include "gjk.h"
#include <stdint.h>
#include <stdbool.h>

#define GRAVITY_CONSTANT    -9.8f

enum collision_layers {
    COLLISION_LAYER_TANGIBLE = (1 << 0),
    COLLISION_LAYER_DAMAGE_PLAYER = (1 << 1),
    COLLISION_LAYER_DAMAGE_ENEMY = (1 << 2),
    COLLISION_LAYER_PLATFORM = (1 << 3),
};

enum collision_group {
    COLLISION_GROUP_PLAYER = 1,
};

typedef void (*bounding_box_calculator)(void* data, struct Quaternion* rotation, struct AABB* box);

typedef enum dynamic_object_type_id {
    DYNAMIC_OBJECT_TYPE_SPHERE,
    DYNAMIC_OBJECT_TYPE_CAPSULE,
    DYNAMIC_OBJECT_TYPE_BOX,
    DYNAMIC_OBJECT_TYPE_CONE,
    DYNAMIC_OBJECT_TYPE_CYLINDER,
    DYNAMIC_OBJECT_TYPE_SWEEP,
} dynamic_object_type_id;

union dynamic_object_type_data
{
    struct { float radius; } sphere;
    struct { float radius; float inner_half_height; } capsule;
    struct { struct Vector3 half_size; } box;
    struct { float radius; float height; } cone;
    struct { float radius; float half_height; } cylinder;
    struct { struct Vector2 range; float radius; float half_height; } sweep;
};

struct dynamic_object_type {
    MinkowskiSum minkowski_sum;
    bounding_box_calculator bounding_box;
    union dynamic_object_type_data data;
    dynamic_object_type_id type;
    float bounce;
    float friction;
};

struct dynamic_object {
    entity_id entity_id;
    struct dynamic_object_type* type;
    struct Vector3* position;
    struct Vector3 prev_position;
    struct Quaternion* rotation_quat;
    struct Vector3 center;
    struct Vector3 velocity;
    struct Vector3 acceleration;
    struct AABB bounding_box;
    float time_scalar;
    float mass;
    float mass_inv;
    uint16_t has_gravity: 1;
    uint16_t is_trigger: 1;
    uint16_t is_fixed: 1;
    uint16_t is_grounded: 1;
    uint16_t is_out_of_bounds: 1;
    uint16_t collision_layers;
    uint16_t collision_group;
    struct contact* active_contacts;
    NodeProxy aabb_tree_node;
};

void dynamic_object_init(
    entity_id entity_id,
    struct dynamic_object* object, 
    struct dynamic_object_type* type,
    uint16_t collision_layers,
    struct Vector3* position, 
    struct Quaternion* rotation,
    float mass
);

void dynamic_object_update(struct dynamic_object* object);

struct contact* dynamic_object_nearest_contact(struct dynamic_object* object);
bool dynamic_object_is_touching(struct dynamic_object* object, entity_id id);

void dynamic_object_accelerate(struct dynamic_object* object, struct Vector3* acceleration);
void dynamic_object_translate_no_force(struct dynamic_object* object, struct Vector3* translation);
void dynamic_object_position_no_force(struct dynamic_object* object, struct Vector3* position);
struct Vector3 dynamic_object_get_velocity(struct dynamic_object* object);
void dynamic_object_set_velocity(struct dynamic_object* object, struct Vector3* velocity);
void dynamic_object_apply_impulse(struct dynamic_object* object, struct Vector3* impulse);

void dynamic_object_apply_constraints(struct dynamic_object* object);

void dynamic_object_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output);
void dynamic_object_recalc_bb(struct dynamic_object* object);

#endif