#ifndef __COLLISION_PHYSICS_OBJECT_H__
#define __COLLISION_PHYSICS_OBJECT_H__

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
#define TERMINAL_VELOCITY   50.0f

enum collision_layers {
    COLLISION_LAYER_TANGIBLE = (1 << 0),
    COLLISION_LAYER_DAMAGE_PLAYER = (1 << 1),
    COLLISION_LAYER_DAMAGE_ENEMY = (1 << 2),
    COLLISION_LAYER_COLLECTABLES = (1 << 3),
};

enum collision_group {
    COLLISION_GROUP_PLAYER = 1,
    COLLISION_GROUP_COLLECTABLE = 2,
};

typedef void (*bounding_box_calculator)(void* data, Quaternion* rotation, AABB* box);

typedef enum physics_object_collision_shape_type {
    COLLISION_SHAPE_SPHERE,
    COLLISION_SHAPE_CAPSULE,
    COLLISION_SHAPE_BOX,
    COLLISION_SHAPE_CONE,
    COLLISION_SHAPE_CYLINDER,
    COLLISION_SHAPE_SWEEP,
} physics_object_collision_shape_type;

union physics_object_collision_shape_data
{
    struct { float radius; } sphere;
    struct { float radius; float inner_half_height; } capsule;
    struct { Vector3 half_size; } box;
    struct { float radius; float half_height; } cone;
    struct { float radius; float half_height; } cylinder;
    struct { Vector2 range; float radius; float half_height; } sweep;
};

struct physics_object_collision_data {
    gjk_support_function gjk_support_function;
    bounding_box_calculator bounding_box_calculator;
    union physics_object_collision_shape_data shape_data;
    Vector3 collider_world_center;
    physics_object_collision_shape_type shape_type;
    float bounce;
    float friction;
};

struct physics_object {
    entity_id entity_id;
    struct physics_object_collision_data* collision; // information about the collision shape
    Vector3* position;
    Vector3 _prev_step_pos;
    Quaternion* rotation;
    Quaternion _prev_step_rot;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 center_offset; // offset from the origin of the object to the center of the collision shape
    AABB bounding_box; // the bounding box fitting the object collider, used for broad phase collision detection
    float time_scalar; // a scalar to adjust the time step for the object, default is 1.0
    float mass; // the mass of the object, cannot be zero
    float gravity_scalar; // how much gravity affects the object, default is 1.0
    uint8_t has_gravity: 1;
    uint8_t is_trigger: 1;
    uint8_t is_fixed: 1;
    uint8_t is_grounded: 1;
    uint8_t is_out_of_bounds: 1;
    uint8_t is_sleeping: 1;
    uint16_t _sleep_counter;
    uint16_t collision_layers; // objects that share at least one layer can collide
    uint16_t collision_group; // objects of the same group do not collide
    struct contact* active_contacts; // contacts with other objects from the last physics step
    NodeProxy _aabb_tree_node_id; // the node id of the object in the phys-object AABB tree of the collision scene
};

void physics_object_init(
    entity_id entity_id,
    struct physics_object* object, 
    struct physics_object_collision_data* collision,
    uint16_t collision_layers,
    Vector3* position, 
    Quaternion* rotation,
    float mass
);

void physics_object_update_euler(struct physics_object* object);
void physics_object_update_velocity_verlet_simple(struct physics_object* object);

struct contact* physics_object_nearest_contact(struct physics_object* object);
bool physics_object_is_touching(struct physics_object* object, entity_id id);

void physics_object_accelerate(struct physics_object* object, Vector3* acceleration);
void physics_object_translate_no_force(struct physics_object* object, Vector3* translation);
void physics_object_position_no_force(struct physics_object* object, Vector3* position);
Vector3 physics_object_get_velocity_verlet(struct physics_object* object);
void physics_object_set_velocity(struct physics_object* object, Vector3* velocity);
void physics_object_apply_impulse(struct physics_object* object, Vector3* impulse);

void physics_object_apply_constraints(struct physics_object* object);

void physics_object_gjk_support_function(void* data, Vector3* direction, Vector3* output);
void physics_object_recalculate_aabb(struct physics_object* object);

#endif