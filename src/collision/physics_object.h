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

#define PHYS_GLOBAL_GRAVITY_MULT 2.5f
#define PHYS_GRAVITY_CONSTANT    -9.8f * PHYS_GLOBAL_GRAVITY_MULT // default earth gravity in m/s^2
#define PHYS_OBJECT_TERMINAL_Y_VELOCITY   50.0f // terminal y-velocity

#define PHYS_OBJECT_SLEEP_THRESHOLD 0.0001f // the amount the object needs to move in one step to be considered in motion
#define PHYS_OBJECT_SLEEP_THRESHOLD_SQ (PHYS_OBJECT_SLEEP_THRESHOLD * PHYS_OBJECT_SLEEP_THRESHOLD)
#define PHYS_OBJECT_SLEEP_STEPS 20 // number of steps the object has to be still before it goes to sleep

#define PHYS_OBJECT_ANGULAR_SLEEP_THRESHOLD 0.001f // angular velocity threshold for sleep (rad/s)
#define PHYS_OBJECT_ANGULAR_SLEEP_THRESHOLD_SQ (PHYS_OBJECT_ANGULAR_SLEEP_THRESHOLD * PHYS_OBJECT_ANGULAR_SLEEP_THRESHOLD)

enum collision_layer {
    COLLISION_LAYER_NONE = 0,
    COLLISION_LAYER_TANGIBLE = (1 << 0),
    COLLISION_LAYER_PLAYER = (1 << 1),
    COLLISION_LAYER_DAMAGE_ENEMY = (1 << 2),
    COLLISION_LAYER_COLLECTABLES = (1 << 3),
    COLLISION_LAYER_TERRAIN_LIKE = (1 << 4),
    COLLISION_LAYER_ALL = 0xff
};

enum collision_group {
    COLLISION_GROUP_NONE = 0,
    COLLISION_GROUP_PLAYER = 1,
    COLLISION_GROUP_COLLECTABLE = 2,
    COLLISION_GROUP_ALL = 0xff
};

typedef void (*bounding_box_calculator)(void* data, Quaternion* rotation, AABB* box);
typedef void (*inertia_calculator)(void* data, float mass, Vector3* out);

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
    inertia_calculator inertia_calculator;
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
    bool has_gravity: true;
    bool is_trigger: true;
    bool is_kinematic: true;
    bool is_rotation_fixed: true;
    bool is_grounded: true;
    bool is_sleeping: true;
    uint16_t _sleep_counter;
    uint16_t collision_layers; // objects that share at least one layer can collide
    uint16_t collision_group; // objects of the same group do not collide
    struct contact* active_contacts; // contacts with other objects from the last physics step
    NodeProxy _aabb_tree_node_id; // the node id of the object in the phys-object AABB tree of the collision scene
    Vector3 angular_velocity;
    Vector3 _torque_accumulator;
    Vector3 _local_inertia_tensor;
    Vector3 _inv_local_intertia_tensor;
    float angular_drag;
};

void physics_object_init(
    entity_id entity_id,
    struct physics_object* object, 
    struct physics_object_collision_data* collision,
    uint16_t collision_layers,
    Vector3* position, 
    Quaternion* rotation,
    Vector3 center_offset,
    float mass
);

void physics_object_update_euler(struct physics_object* object);
void physics_object_update_velocity_verlet(struct physics_object* object);
void physics_object_update_angular_velocity(struct physics_object* object);

struct contact* physics_object_nearest_contact(struct physics_object* object);
bool physics_object_is_touching(struct physics_object* object, entity_id id);

void physics_object_accelerate(struct physics_object* object, Vector3* acceleration);
void physics_object_translate_no_force(struct physics_object* object, Vector3* translation);
void physics_object_position_no_force(struct physics_object* object, Vector3* position);
void physics_object_set_velocity(struct physics_object* object, Vector3* velocity);
void physics_object_apply_impulse(struct physics_object* object, Vector3* impulse);

void physics_object_apply_torque(struct physics_object* object, Vector3* torque);
void physics_object_apply_angular_impulse(struct physics_object* object, Vector3* angular_impulse);
void physics_object_apply_force_at_point(struct physics_object* object, Vector3* force, Vector3* world_point);
void physics_object_set_angular_velocity(struct physics_object* object, Vector3* angular_velocity);

void physics_object_apply_constraints(struct physics_object* object);

void physics_object_gjk_support_function(void* data, Vector3* direction, Vector3* output);
void physics_object_recalculate_aabb(struct physics_object* object);

#endif