#ifndef __COLLISION_PHYSICS_OBJECT_H__
#define __COLLISION_PHYSICS_OBJECT_H__

#include "../entity/entity_id.h"
#include "../math/vector3.h"
#include "../math/vector2.h"
#include "../math/aabb.h"
#include "../math/quaternion.h"
#include "../collision/aabb_tree.h"
#include "contact.h"
#include "gjk.h"
#include <stdint.h>
#include <stdbool.h>

#define PHYS_GLOBAL_GRAVITY_MULT 1.0f // adjust this according to general world scale
#define PHYS_GRAVITY_CONSTANT    -9.8f * PHYS_GLOBAL_GRAVITY_MULT // default gravity in m/s^2

#define PHYS_OBJECT_TERMINAL_SPEED   90.0f // terminal linear speed / velocity magnitude (units/s)
#define PHYS_OBJECT_TERMINAL_ANGULAR_SPEED 50.0f // terminal angular speed / angular velocity magnitude (rad/s)
#define PHYS_OBJECT_TERMINAL_ANGULAR_SPEED_SQ (PHYS_OBJECT_TERMINAL_ANGULAR_SPEED * PHYS_OBJECT_TERMINAL_ANGULAR_SPEED)

// The minimum amount of units an object has to move between physics ticks to be considered to have changed position
#define PHYS_OBJECT_POS_CHANGE_SLEEP_THRESHOLD 0.013f // the amount the object needs to move in one step to be considered in motion
#define PHYS_OBJECT_POS_CHANGE_SLEEP_THRESHOLD_SQ (PHYS_OBJECT_POS_CHANGE_SLEEP_THRESHOLD * PHYS_OBJECT_POS_CHANGE_SLEEP_THRESHOLD)

// The minimum speed an object needs to have to be considered in motion
// Keep in mind the speed is in units/second not units/physics_tick
#define PHYS_OBJECT_SPEED_SLEEP_THRESHOLD 0.6f 
#define PHYS_OBJECT_SPEED_SLEEP_THRESHOLD_SQ (PHYS_OBJECT_SPEED_SLEEP_THRESHOLD * PHYS_OBJECT_SPEED_SLEEP_THRESHOLD)

// The similarity of quaternion rotations between physics ticks to consider the rotation unchanged
#define PHYS_OBJECT_ROT_SIMILARITY_SLEEP_THRESHOLD 0.999999f

// The minimum angular speed an object needs to have to be considered rotating
// Keep in mind the speed is in rad/second not rad/physics_tick
#define PHYS_OBJECT_ANGULAR_CHANGE_SLEEP_THRESHOLD 0.1f
#define PHYS_OBJECT_ANGULAR_CHANGE_SLEEP_THRESHOLD_SQ (PHYS_OBJECT_ANGULAR_CHANGE_SLEEP_THRESHOLD * PHYS_OBJECT_ANGULAR_CHANGE_SLEEP_THRESHOLD)

// The threshold for the angular speed (in rad/sec) under which the angular velocity will be dampened more aggressively
// to reduce tiny rotations
#define PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD 0.1f
#define PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD_SQ (PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD * PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD)
#define PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD_SQ_INV (1.0f / PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD_SQ)

#define PHYS_OBJECT_SLEEP_STEPS 20 // number of timesteps the object has to be still for before it goes to sleep


/// @brief Enum of collision layers a physics object can be part of or interact with
enum collision_layer {
    COLLISION_LAYER_NONE = 0,
    COLLISION_LAYER_TANGIBLE = (1 << 0),
    COLLISION_LAYER_PLAYER = (1 << 1),
    COLLISION_LAYER_DAMAGE_ENEMY = (1 << 2),
    COLLISION_LAYER_COLLECTABLES = (1 << 3),
    COLLISION_LAYER_TERRAIN_LIKE = (1 << 4),
    COLLISION_LAYER_ALL = 0xff
};

/// @brief Enum of collision groups. Physics objects that are part of a collision group cannot collide/interact with objects of the same group
enum collision_group {
    COLLISION_GROUP_NONE = 0,
    COLLISION_GROUP_PLAYER = 1,
    COLLISION_GROUP_COLLECTABLE = 2,
    COLLISION_GROUP_ALL = 0xff
};

/// @brief Defines a function pointer for a general bounding_box_calculater function. These functions are implemented for each collider shape.
typedef void (*bounding_box_calculator)(const void* data, const Quaternion* rotation, AABB* box);

/// @brief Defines a function pointer for a inertia_calculator function. These functions are implemented per collision shape.
typedef void (*inertia_calculator)(void* data, Vector3* out);

/// @brief Enum of possible collision shapes
typedef enum physics_object_collision_shape_type {
    COLLISION_SHAPE_SPHERE,
    COLLISION_SHAPE_CAPSULE,
    COLLISION_SHAPE_BOX,
    COLLISION_SHAPE_CONE,
    COLLISION_SHAPE_CYLINDER,
    COLLISION_SHAPE_SWEEP,
} physics_object_collision_shape_type;

/// @brief Flags for physics_object constraints
///
/// @note Position constraints are applied in World space, and rotation constraints are applied in the inertia space (local)
typedef enum physics_object_constraints {
    CONSTRAINTS_NONE = 0,
    CONSTRAINTS_FREEZE_POSITION_X = (1 << 0),
    CONSTRAINTS_FREEZE_POSITION_Y = (1 << 1),
    CONSTRAINTS_FREEZE_POSITION_Z = (1 << 2),
    CONSTRAINTS_FREEZE_POSITION_ALL = (CONSTRAINTS_FREEZE_POSITION_X | CONSTRAINTS_FREEZE_POSITION_Y | CONSTRAINTS_FREEZE_POSITION_Z),
    CONSTRAINTS_FREEZE_ROTATION_X = (1 << 3),
    CONSTRAINTS_FREEZE_ROTATION_Y = (1 << 4),
    CONSTRAINTS_FREEZE_ROTATION_Z = (1 << 5),
    CONSTRAINTS_FREEZE_ROTATION_ALL = (CONSTRAINTS_FREEZE_ROTATION_X | CONSTRAINTS_FREEZE_ROTATION_Y | CONSTRAINTS_FREEZE_ROTATION_Z),
    CONSTRAINTS_ALL = 0xff
} physics_object_constraints;

/// @brief Defines the parameters necessary to describe a collision shape
union physics_object_collision_shape_data
{
    struct { float radius; } sphere;
    struct { float radius; float inner_half_height; } capsule;
    struct { Vector3 half_size; } box;
    struct { float radius; float half_height; } cone;
    struct { float radius; float half_height; } cylinder;
    struct { Vector2 range; float radius; float half_height; } sweep;
};

/// @brief Defines a set of functions and data to describe a collider.
///
/// @note Most collider primitives offer a #define function to assemble this
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

/// @brief 
typedef struct physics_object {
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
    float _mass; // the mass of the object, cannot be zero - change only via physics_object_set_mass!
    float _inv_mass; //must be recalculated if mass changes!
    float gravity_scalar; // how much gravity affects the object, default is 1.0
    bool has_gravity: true;
    bool is_trigger: true;
    bool is_kinematic: true;
    bool is_grounded: true;
    bool _is_sleeping: true;
    uint16_t constraints; // flags that control which degrees of freedom are allowed for the simulation of this object
    uint16_t _sleep_counter;
    uint16_t collision_layers; // objects that share at least one layer can collide
    uint16_t collision_group; // objects of the same group do not collide
    contact* active_contacts; // contacts with other objects from the last physics step
    node_proxy _aabb_tree_node_id; // the node id of the object in the phys-object AABB tree of the collision scene
    Vector3 angular_velocity;
    Vector3 _torque_accumulator;
    Vector3 _local_inertia_tensor; // must be recalculated if mass or collision changes!
    Vector3 _inv_local_intertia_tensor; // must be recalculated if _local_inertia_tensor changes!
    float _inv_world_inertia_tensor[9]; // 3x3 matrix, recalculated every frame
    float angular_damping; // defines the decay rate of an objects angular velocity. Higher = object rotation slows down faster.
    float _prev_angular_speed_sq;
    float _ground_support_factor;
} physics_object;


/// @brief Initializes a physics object with the given parameters.
/// @param entity_id The entity ID of the owner entity of this physics object.
/// @param object Pointer to the object to be initialized.
/// @param collision Pointer to the objects collision information. Must be set!
/// @param collision_layers Flags that define which layers the object can collide/interact with. See enum collision_layer
/// @param position Pointer to the 3D vector that represents the objects position (usually shared with the owners). Must be set!
/// @param rotation Pointer to the quaternion rotation that represents the objects rotation (usually shared with owner). May be NULL.
/// @param center_offset 3D vector that describes how the phys objects collision is offset from the position (eg if the model orign is not at center)
/// @param mass The mass of the physics object. Must be > 0.
void physics_object_init(
    entity_id entity_id,
    physics_object* object, 
    struct physics_object_collision_data* collision,
    uint16_t collision_layers,
    Vector3* position, 
    Quaternion* rotation,
    Vector3 center_offset,
    float mass
);



/// @brief iterate through the active contacts of the object and return the contact with the smallest distance to the object
/// @param object the physics_object whose contacts are to be checked
/// @return the contact with the smallest distance to the object
contact* physics_object_nearest_contact(physics_object* object);


/// @brief iterate through the active contacts of the object and check if the given entity id is in the list
/// @param object the physics_object whose contacts are to be checked
/// @param id the entity id of the object to check for
/// @return true if the object is in the list of contacts, false otherwise
bool physics_object_is_touching(physics_object* object, entity_id id);


/// @brief Sets the mass of a physics_object and recalculates mass-dependent properties.
///
/// This includes _inv_mass, _local_inertia_tensor & _inv_local_inertia_tensor.
///
/// Use this function if the mass of a physics object must be adjusted after initialization instead of modifying the mass directly.
/// @param object 
/// @param new_mass 
void physics_object_set_mass(physics_object* object, float new_mass);

// -------- FORCE --------

/// @brief Apply a force at a specific world point, generating both linear and angular effects
/// @param object
/// @param force the force vector in world space
/// @param world_point the point in world space where the force is applied
void physics_object_apply_force_at_point(physics_object* object, Vector3* force, Vector3* world_point);


// -------- LINEAR --------

/// @brief Integrate acceleration into velocity (first half of semi-implicit Euler)
/// @param object
void physics_object_integrate_velocity(physics_object* object);

/// @brief Integrate torque into angular velocity
/// @param object
void physics_object_integrate_angular_velocity(physics_object* object);

/// @brief Integrate velocity into position (second half of semi-implicit Euler)
/// @param object
void physics_object_integrate_position(physics_object* object);

/// @brief Integrate angular velocity into rotation
/// @param object
void physics_object_integrate_rotation(physics_object* object);

/// @brief Accelerates the object by the given acceleration vector. 
/// @param object 
/// @param acceleration 
void physics_object_accelerate(physics_object* object, Vector3* acceleration);


/// @brief Manually set the velocity of the object to the given velocity vector
/// @param object 
/// @param velocity 
void physics_object_set_velocity(physics_object* object, Vector3* velocity);


/// @brief apply a linear impulse force to the object
/// @param object
/// @param impulse linear impulse in world space (N⋅m⋅s)
void physics_object_apply_linear_impulse(physics_object* object, Vector3* impulse);


// -------- ANGULAR --------


/// @brief Apply torque to the object (accumulated and applied during physics update)
/// @param object
/// @param torque the torque vector in world space (N⋅m)
void physics_object_apply_torque(physics_object* object, Vector3* torque);


/// @brief Apply an angular impulse directly to angular velocity.
///
/// This will take into account the objects rotational constraints!
/// @param object
/// @param angular_impulse angular impulse in world space (N⋅m⋅s)
void physics_object_apply_angular_impulse(physics_object* object, Vector3* angular_impulse);


/// @brief Manually set the angular velocity of the object
/// @param object
/// @param angular_velocity the angular velocity in world space (rad/s)
void physics_object_set_angular_velocity(physics_object* object, Vector3* angular_velocity);


/// @brief Enforce the objects position to stay in reasonable bounds as to not crash when converting the floating point position to the fixed matrix for rendering
/// @param object 
void physics_object_apply_position_constraints(physics_object* object);

/// @brief will transform the given direction vector into the local space of the object and then call the GJK support function associated with the object
/// @param data expected to be a pointer to a physics_object
/// @param direction the direction vector in world space
/// @param output the resulting Support point in world space
void physics_object_gjk_support_function(const void* data, const Vector3* direction, Vector3* output);

/// @brief re-caluclates the bounding box of the object using the collision type's bounding box function.
///
/// This will also take into account the center_offset and rotation of the object.
/// @param object
void physics_object_recalculate_aabb(physics_object* object);

/// @brief Updates the world space inverse inertia tensor based on current rotation
/// @param object
void physics_object_update_world_inertia(physics_object* object);

/// @brief Wakes up the object (resets sleep timer and state)
/// @param object
void physics_object_wake(physics_object* object);

#endif