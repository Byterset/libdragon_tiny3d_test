#include "physics_object.h"
#include <assert.h>
#include "../time/time.h"
#include "../math/minmax.h"
#include <math.h>
#include <stddef.h>

/// @brief Initializes a physics object with the given parameters
/// @param entity_id 
/// @param object 
/// @param collision 
/// @param collision_layers 
/// @param position 
/// @param rotation 
/// @param center_offset
/// @param mass 
void physics_object_init(
    entity_id entity_id,
    struct physics_object* object, 
    struct physics_object_collision_data* collision,
    uint16_t collision_layers,
    Vector3* position, 
    Quaternion* rotation,
    Vector3 center_offset,
    float mass
) {
    assert(mass > 0.0f);
    object->entity_id = entity_id;
    object->collision = collision;
    object->position = position;
    object->_prev_step_pos = *position;
    object->rotation = rotation;
    object->_prev_step_rot = gQuaternionZero;
    object->velocity = gZeroVec;
    object->center_offset = center_offset;
    object->time_scalar = 1.0f;
    object->gravity_scalar = 1.0f;
    object->mass = mass;
    object->has_gravity = true;
    object->is_trigger = false;
    object->is_kinematic = false;
    object->is_grounded = false;
    object->_is_sleeping = false;
    object->constrain_movement_x = false;
    object->constrain_movement_y = false;
    object->constrain_movement_z = false;
    object->constrain_rotation_x = false;
    object->constrain_rotation_y = false;
    object->constrain_rotation_z = false;
    object->collision_layers = collision_layers;
    object->collision_group = 0;
    object->active_contacts = 0;
    object->angular_drag = 0.02f;
    object->angular_velocity = gZeroVec;
    object->_torque_accumulator = gZeroVec;

    // Calculate inertia tensor if inertia calculator is provided
    if (collision->inertia_calculator) {
        collision->inertia_calculator(object, mass, &object->_local_inertia_tensor);
    } else {
        // Fallback: treat as unit sphere if no calculator provided
        float default_inertia = 0.4f * mass;
        object->_local_inertia_tensor = (Vector3){{default_inertia, default_inertia, default_inertia}};
    }
    object->_inv_local_intertia_tensor.x = 1 / object->_local_inertia_tensor.x;
    object->_inv_local_intertia_tensor.y = 1 / object->_local_inertia_tensor.y;
    object->_inv_local_intertia_tensor.z = 1 / object->_local_inertia_tensor.z;

    physics_object_recalculate_aabb(object);
}

void physics_object_update_euler(struct physics_object* object) {
    if (object->is_trigger || object->is_kinematic) {
        return;
    }

    vector3AddScaled(&object->velocity, &object->acceleration, FIXED_DELTATIME * object->time_scalar, &object->velocity);
    object->velocity.y = clampf(object->velocity.y, -PHYS_OBJECT_TERMINAL_Y_VELOCITY, PHYS_OBJECT_TERMINAL_Y_VELOCITY);

    // Apply movement constraints
    if (object->constrain_movement_x) object->velocity.x = 0.0f;
    if (object->constrain_movement_y) object->velocity.y = 0.0f;
    if (object->constrain_movement_z) object->velocity.z = 0.0f;

    object->is_grounded = false;

    vector3AddScaled(object->position, &object->velocity, FIXED_DELTATIME * object->time_scalar, object->position);
    object->acceleration = gZeroVec;

}

void physics_object_update_implicit_euler(struct physics_object* o) {
    if (o->is_trigger || o->is_kinematic) return;

    // Implicit Euler: v_{t+1} = v_t + a_{t+1} * dt
    //                 x_{t+1} = x_t + v_{t+1} * dt
    // Here we assume acceleration is based on known forces at current step.

    // Update velocity first
    vector3AddScaled(&o->velocity, &o->acceleration, FIXED_DELTATIME, &o->velocity);
    o->velocity.y = clampf(o->velocity.y, -PHYS_OBJECT_TERMINAL_Y_VELOCITY, PHYS_OBJECT_TERMINAL_Y_VELOCITY);

    // Apply movement constraints
    if (o->constrain_movement_x) o->velocity.x = 0.0f;
    if (o->constrain_movement_y) o->velocity.y = 0.0f;
    if (o->constrain_movement_z) o->velocity.z = 0.0f;

    // Update position using new velocity
    vector3AddScaled(o->position, &o->velocity, FIXED_DELTATIME, o->position);


    o->acceleration = gZeroVec;
    o->is_grounded = false;
}

void physics_object_update_velocity_verlet(struct physics_object* o) {
    if (o->is_trigger || o->is_kinematic) return;

    vector3AddScaled(o->position, &o->velocity, FIXED_DELTATIME, o->position);
    vector3AddScaled(o->position, &o->acceleration, 0.5f * FIXED_DELTATIME * FIXED_DELTATIME, o->position);

    vector3AddScaled(&o->velocity, &o->acceleration, FIXED_DELTATIME, &o->velocity);
    o->velocity.y = clampf(o->velocity.y, -PHYS_OBJECT_TERMINAL_Y_VELOCITY, PHYS_OBJECT_TERMINAL_Y_VELOCITY);

    // Apply movement constraints
    if (o->constrain_movement_x) o->velocity.x = 0.0f;
    if (o->constrain_movement_y) o->velocity.y = 0.0f;
    if (o->constrain_movement_z) o->velocity.z = 0.0f;

    o->acceleration = gZeroVec;
    o->is_grounded = false;
}

void physics_object_update_angular_velocity(struct physics_object* object) {
    // Skip if trigger, kinematic, or no rotation quaternion
    if (object->is_trigger || object->is_kinematic || !object->rotation) {
        return;
    }

    // Skip if all rotation axes are constrained
    if (object->constrain_rotation_x && object->constrain_rotation_y && object->constrain_rotation_z) {
        return;
    }

    // Skip if no angular motion
    if (vector3IsZero(&object->angular_velocity) && vector3IsZero(&object->_torque_accumulator)) {
        return;
    }

    // Calculate angular acceleration: α = I^-1 * τ
    // Since we store diagonal inertia tensor, inverse is just 1/Ixx, 1/Iyy, 1/Izz
    if (!vector3IsZero(&object->_torque_accumulator)) {
        Vector3 angular_acceleration;
        angular_acceleration.x = object->_torque_accumulator.x * object->_inv_local_intertia_tensor.x;
        angular_acceleration.y = object->_torque_accumulator.y * object->_inv_local_intertia_tensor.y;
        angular_acceleration.z = object->_torque_accumulator.z * object->_inv_local_intertia_tensor.z;

        // Apply per-axis constraints to angular acceleration
        if (object->constrain_rotation_x) angular_acceleration.x = 0.0f;
        if (object->constrain_rotation_y) angular_acceleration.y = 0.0f;
        if (object->constrain_rotation_z) angular_acceleration.z = 0.0f;

        // Update angular velocity: ω = ω + α * dt
        vector3AddScaled(&object->angular_velocity, &angular_acceleration,
                        FIXED_DELTATIME * object->time_scalar, &object->angular_velocity);

        // Clear torque accumulator
        object->_torque_accumulator = gZeroVec;
    }

    // Apply per-axis constraints to angular velocity
    if (object->constrain_rotation_x) object->angular_velocity.x = 0.0f;
    if (object->constrain_rotation_y) object->angular_velocity.y = 0.0f;
    if (object->constrain_rotation_z) object->angular_velocity.z = 0.0f;

    // Apply angular damping
    if (object->angular_drag > 0.0f) {
        // Normal damping for faster rotations
        float damping_factor = 1.0f - object->angular_drag;
        if (damping_factor < 0.0f) damping_factor = 0.0f;
        vector3Scale(&object->angular_velocity, &object->angular_velocity, damping_factor);
    }

    // Calculate rotated center offset before rotation
    Vector3 center_offset_old;
    quatMultVector(object->rotation, &object->center_offset, &center_offset_old);

    // Apply angular velocity to rotation quaternion
    quatApplyAngularVelocity(object->rotation, &object->angular_velocity,
                            FIXED_DELTATIME * object->time_scalar, object->rotation);
    quatNormalize(object->rotation, object->rotation);

    // Calculate rotated center offset after rotation
    Vector3 center_offset_new;
    quatMultVector(object->rotation, &object->center_offset, &center_offset_new);

    // Adjust position so center of mass stays in same world location
    // This makes rotation happen around center of mass, not object origin
    // position_adjustment = old_offset - new_offset
    Vector3 position_adjustment;
    vector3Sub(&center_offset_old, &center_offset_new, &position_adjustment);
    vector3Add(object->position, &position_adjustment, object->position);
}

void physics_object_apply_constraints(struct physics_object* object){
    if (object->position->y <= -20){
        *object->position = (Vector3){{0, 20, 0}};
        object->velocity = gZeroVec;
    }
    if (object->position->y >= 2000){
        object->position->y = 2000;
        object->velocity.y = 0;
    }
    if (object->position->x <= -2000){
        object->position->x = -2000;
        object->velocity.x = 0;
    }
    if (object->position->x >= 2000){
        object->position->x = 2000;
        object->velocity.x = 0;
    }
    if (object->position->z <= -2000){
        object->position->z = -2000;
        object->velocity.z = 0;
    }
    if (object->position->z >= 2000){
        object->position->z = 2000;
        object->velocity.z = 0;
    }
}

/// @brief Accelerates the object by the given acceleration vector. 
/// @param object 
/// @param acceleration 
void physics_object_accelerate(struct physics_object* object, Vector3* acceleration) {
    vector3Add(&object->acceleration, acceleration, &object->acceleration);
}

/// @brief Set the velocity of the object to the given velocity vector
/// @param object 
/// @param velocity 
void physics_object_set_velocity(struct physics_object* object, Vector3* velocity){
    vector3Copy(velocity, &object->velocity);
}

/// @brief apply and impulse force to the object
/// @param object
/// @param impulse
void physics_object_apply_impulse(struct physics_object* object, Vector3* impulse) {
    vector3AddScaled(&object->velocity, impulse, 1.0f / object->mass, &object->velocity);
}

/// @brief Apply torque to the object (accumulated and applied during physics update)
/// @param object
/// @param torque the torque vector in world space (N⋅m)
void physics_object_apply_torque(struct physics_object* object, Vector3* torque) {
    vector3Add(&object->_torque_accumulator, torque, &object->_torque_accumulator);
}

/// @brief Apply an angular impulse directly to angular velocity
/// @param object
/// @param angular_impulse angular impulse in world space (N⋅m⋅s)
void physics_object_apply_angular_impulse(struct physics_object* object, Vector3* angular_impulse) {
    if (object->is_kinematic || !object->rotation) {
        return;
    }

    // Skip if all rotation axes are constrained
    if (object->constrain_rotation_x && object->constrain_rotation_y && object->constrain_rotation_z) {
        return;
    }

    // Δω = I^-1 * angular_impulse
    // Since we store diagonal inertia tensor, inverse is just 1/Ixx, 1/Iyy, 1/Izz
    Vector3 angular_velocity_change;
    angular_velocity_change.x = angular_impulse->x * object->_inv_local_intertia_tensor.x;
    angular_velocity_change.y = angular_impulse->y * object->_inv_local_intertia_tensor.y;
    angular_velocity_change.z = angular_impulse->z * object->_inv_local_intertia_tensor.z;

    // Apply per-axis constraints
    if (object->constrain_rotation_x) angular_velocity_change.x = 0.0f;
    if (object->constrain_rotation_y) angular_velocity_change.y = 0.0f;
    if (object->constrain_rotation_z) angular_velocity_change.z = 0.0f;

    vector3Add(&object->angular_velocity, &angular_velocity_change, &object->angular_velocity);
}

/// @brief Apply a force at a specific world point, generating both linear and angular effects
/// @param object
/// @param force the force vector in world space
/// @param world_point the point in world space where the force is applied
void physics_object_apply_force_at_point(struct physics_object* object, Vector3* force, Vector3* world_point) {
    // Apply linear force
    vector3AddScaled(&object->acceleration, force, 1.0f / object->mass, &object->acceleration);

    // Calculate torque: τ = r × F
    // r is the vector from center of mass to the point of application
    Vector3 center_of_mass;
    if (object->rotation) {
        Vector3 rotated_offset;
        quatMultVector(object->rotation, &object->center_offset, &rotated_offset);
        vector3Add(object->position, &rotated_offset, &center_of_mass);
    } else {
        vector3Add(object->position, &object->center_offset, &center_of_mass);
    }

    Vector3 r;
    vector3Sub(world_point, &center_of_mass, &r);

    Vector3 torque;
    vector3Cross(&r, force, &torque);

    physics_object_apply_torque(object, &torque);
}

/// @brief Set the angular velocity of the object
/// @param object
/// @param angular_velocity the angular velocity in world space (rad/s)
void physics_object_set_angular_velocity(struct physics_object* object, Vector3* angular_velocity) {
    vector3Copy(angular_velocity, &object->angular_velocity);
}

/// @brief iterate through the active contacts of the object and return the contact with the smallest distance to the object
/// @param object the physics_object whose contacts are to be checked
/// @return the contact with the smallest distance to the object
struct contact* physics_object_nearest_contact(struct physics_object* object) {
    struct contact* nearest_target = NULL;
    struct contact* current = object->active_contacts;
    float distance = 0.0f;

    while (current) {
        float check = vector3DistSqrd(&current->point, object->position);
        if (!nearest_target || check < distance) {
            distance = check;
            nearest_target = current;
        }

        current = current->next;
    }

    return nearest_target;
}

/// @brief iterate through the active contacts of the object and check if the given entity id is in the list
/// @param object the physics_object whose contacts are to be checked
/// @param id the entity id of the object to check for
/// @return true if the object is in the list of contacts, false otherwise
bool physics_object_is_touching(struct physics_object* object, entity_id id) {
    struct contact* current = object->active_contacts;

    while (current) {
        if (current->other_object == id) {
            return true;
        }
            
        current = current->next;
    }

    return false;
}

/// @brief will transform the given direction vector into the local space of the object and then call the GJK support function associated with the object
/// @param data expected to be a pointer to a physics_object
/// @param direction the direction vector in world space
/// @param output the resulting Support point in world space
void physics_object_gjk_support_function(void* data, Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    Vector3 world_center;
    Vector3 localDir;
    if(object->rotation){
        Quaternion inv_rotation;
        quatConjugate(object->rotation, &inv_rotation);
        quatMultVector(&inv_rotation, direction, &localDir);
        quatMultVector(object->rotation, &object->center_offset, &world_center);
    }
    else{
        vector3Copy(direction, &localDir);
        vector3Copy(&object->center_offset, &world_center);
    }

    vector3Normalize(&localDir, &localDir);
    object->collision->gjk_support_function(object, &localDir, output);

    if(object->rotation){
        quatMultVector(object->rotation, output, output);
    }

    vector3Add(output, object->position, output);
    vector3Add(output, &world_center, output);
}


/// @brief re-caluclates the bounding box of the object using the collision type's bounding box function
/// @param object 
void physics_object_recalculate_aabb(struct physics_object* object) {
    //calculate bounding box for object in local space
    object->collision->bounding_box_calculator(object, object->rotation, &object->bounding_box);
    Vector3 offset;

    //calculate the rotated center offset if the object has a rotation
    if(object->rotation){
        quatMultVector(object->rotation, &object->center_offset, &offset);
    }
    else{
        vector3Copy(&object->center_offset, &offset);
    }

    //calculate world space bounding box limits
    vector3Add(&offset, object->position, &offset);
    vector3Add(&object->bounding_box.min, &offset, &object->bounding_box.min);
    vector3Add(&object->bounding_box.max, &offset, &object->bounding_box.max);
    object->collision->collider_world_center = offset;
}