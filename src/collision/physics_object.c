#include "physics_object.h"
#include <assert.h>
#include "../time/time.h"
#include "../math/minmax.h"
#include <math.h>
#include <stddef.h>

void physics_object_init(
    entity_id entity_id,
    physics_object* object, 
    struct physics_object_collision_data* collision,
    uint16_t collision_layers,
    Vector3* position, 
    Quaternion* rotation,
    Vector3 center_offset,
    float mass
) {
    assertf(mass > 0.0f, "Physics Object Mass cannot be <= 0!");
    assertf(collision, "Physics Object must provide collision information");
    assertf(position, "Physics Object must have a position");
    object->entity_id = entity_id;
    object->collision = collision;
    object->position = position;
    object->_prev_step_pos = *position;
    object->rotation = rotation;
    quatIdent(&object->_prev_step_rot);
    object->velocity = gZeroVec;
    object->center_offset = center_offset;
    object->time_scalar = 1.0f;
    object->gravity_scalar = 1.0f;
    object->_mass = mass;
    object->_inv_mass = 1.0f/mass;
    object->has_gravity = true;
    object->is_trigger = false;
    object->is_kinematic = false;
    object->is_grounded = false;
    object->_is_sleeping = false;
    object->constraints = CONSTRAINTS_NONE;
    object->collision_layers = collision_layers;
    object->collision_group = 0;
    object->active_contacts = 0;
    object->angular_damping = 0.03f * (60.0f/PHYSICS_TICKRATE);
    object->angular_velocity = gZeroVec;
    object->_prev_angular_speed_sq = 0;
    object->_torque_accumulator = gZeroVec;
    object->acceleration = gZeroVec;
    object->_ground_support_factor = 0;

    // Calculate inertia tensor if inertia calculator is provided
    if (collision && collision->inertia_calculator) {
        collision->inertia_calculator(object, &object->_local_inertia_tensor);
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


void physics_object_update_velocity_semi_implicit_euler(physics_object* object) {
    if (object->is_trigger || object->is_kinematic) return;

    // Update velocity first
    vector3AddScaled(&object->velocity, &object->acceleration, FIXED_DELTATIME * object->time_scalar, &object->velocity);

    // Clamp Velocity to maxSpeed
    vector3ClampMag(&object->velocity, &object->velocity, PHYS_OBJECT_TERMINAL_SPEED);

    // Apply movement constraints
    if (object->constraints & CONSTRAINTS_FREEZE_POSITION_X) object->velocity.x = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_POSITION_Y) object->velocity.y = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_POSITION_Z) object->velocity.z = 0.0f;

    // Update position using new velocity
    vector3AddScaled(object->position, &object->velocity, FIXED_DELTATIME  * object->time_scalar, object->position);

    object->acceleration = gZeroVec;
    object->is_grounded = false;
}

void physics_object_integrate_velocity(physics_object* object) {
    if (object->is_trigger || object->is_kinematic) return;

    // Update velocity from acceleration
    vector3AddScaled(&object->velocity, &object->acceleration, FIXED_DELTATIME * object->time_scalar, &object->velocity);

    // Clamp Velocity to maxSpeed
    vector3ClampMag(&object->velocity, &object->velocity, PHYS_OBJECT_TERMINAL_SPEED);

    // Apply movement constraints
    if (object->constraints & CONSTRAINTS_FREEZE_POSITION_X) object->velocity.x = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_POSITION_Y) object->velocity.y = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_POSITION_Z) object->velocity.z = 0.0f;

    object->acceleration = gZeroVec;
}

void physics_object_integrate_position(physics_object* object) {
    if (object->is_trigger || object->is_kinematic) return;

    // Update position using current velocity
    vector3AddScaled(object->position, &object->velocity, FIXED_DELTATIME * object->time_scalar, object->position);

    object->is_grounded = false;
}


void physics_object_update_angular_velocity(physics_object* object) {
    // Skip if trigger, kinematic, or no rotation quaternion
    if (object->is_trigger || object->is_kinematic || !object->rotation) {
        return;
    }

    // Skip if all rotation axes are constrained
    if ((object->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL) {
        return;
    }

    // Skip if no angular motion
    if (vector3IsZero(&object->angular_velocity) && vector3IsZero(&object->_torque_accumulator)) {
        return;
    }

    Quaternion rotation_inverse;
    quatConjugate(object->rotation, &rotation_inverse);

    // Calculate angular acceleration: α = I^-1 * τ
    // Since we store diagonal inertia tensor, inverse is just 1/Ixx, 1/Iyy, 1/Izz
    if (!vector3IsZero(&object->_torque_accumulator)) {
        // Transform torque from world space to local space
        Vector3 local_torque;
        quatMultVector(&rotation_inverse, &object->_torque_accumulator, &local_torque);

        // Calculate local angular acceleration
        Vector3 local_angular_acceleration;
        local_angular_acceleration.x = local_torque.x * object->_inv_local_intertia_tensor.x;
        local_angular_acceleration.y = local_torque.y * object->_inv_local_intertia_tensor.y;
        local_angular_acceleration.z = local_torque.z * object->_inv_local_intertia_tensor.z;

        // Apply per-axis constraints in LOCAL space
        if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_angular_acceleration.x = 0.0f;
        if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_angular_acceleration.y = 0.0f;
        if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_angular_acceleration.z = 0.0f;

        // Transform angular acceleration back to world space
        Vector3 angular_acceleration;
        quatMultVector(object->rotation, &local_angular_acceleration, &angular_acceleration);

        // Update angular velocity: ω = ω + α * dt
        vector3AddScaled(&object->angular_velocity, &angular_acceleration,
                        FIXED_DELTATIME * object->time_scalar, &object->angular_velocity);

        // Clear torque accumulator
        object->_torque_accumulator = gZeroVec;
    }

    // Clamp Angular Velocity to maxRotationalSpeed
    // We don't use vector3ClampMag here because we need the angular_speed_sq later again
    float angular_speed_sq = vector3MagSqrd(&object->angular_velocity);
    if(angular_speed_sq > PHYS_OBJECT_TERMINAL_ANGULAR_SPEED_SQ) {
        float inv_len = 1.0f / sqrtf(angular_speed_sq);
        float scale = PHYS_OBJECT_TERMINAL_ANGULAR_SPEED * inv_len;
        vector3Scale(&object->angular_velocity, &object->angular_velocity, scale);
    }

    // Also apply constraints to angular velocity in local space to catch any numerical drift
    // Transform current angular velocity to local space
    Vector3 local_angular_velocity;
    quatMultVector(&rotation_inverse, &object->angular_velocity, &local_angular_velocity);

    // Apply per-axis constraints in LOCAL space
    if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_angular_velocity.x = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_angular_velocity.y = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_angular_velocity.z = 0.0f;

    // Transform back to world space
    quatMultVector(object->rotation, &local_angular_velocity, &object->angular_velocity);

    // dampen smaller rotations up to a threshold faster
    if (object->angular_damping > 0.0f)
    {

        if (angular_speed_sq < PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD_SQ)
        {
            // damping scales with how close the angular speed is to zero
            float t = angular_speed_sq * PYHS_OBJECT_AMPLIFY_ANG_SPEED_DAMPING_THRESHOLD_SQ_INV; // 0 → 1 as speed goes 0 → threshold
            float damping_factor = 0.8f + 0.2f * t;                                              // Range: 0.85 → 1.0
            vector3Scale(&object->angular_velocity, &object->angular_velocity, damping_factor);
        }
        //for objects that rotate faster than the threshold apply regular damping
        else
        {
            vector3Scale(&object->angular_velocity, &object->angular_velocity, 1.0f - object->angular_damping);
        }
    }
    object->_prev_angular_speed_sq = angular_speed_sq;

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

void physics_object_apply_position_constraints(physics_object* object){
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


void physics_object_accelerate(physics_object* object, Vector3* acceleration) {
    vector3Add(&object->acceleration, acceleration, &object->acceleration);
}


void physics_object_set_velocity(physics_object* object, Vector3* velocity){
    vector3Copy(velocity, &object->velocity);
}


void physics_object_apply_linear_impulse(physics_object* object, Vector3* impulse) {
    vector3AddScaled(&object->velocity, impulse, 1.0f / object->_mass, &object->velocity);
}


void physics_object_apply_torque(physics_object* object, Vector3* torque) {
    vector3Add(&object->_torque_accumulator, torque, &object->_torque_accumulator);
}


void physics_object_apply_angular_impulse(physics_object* object, Vector3* angular_impulse) {
    if (object->is_kinematic || !object->rotation) {
        return;
    }

    // Skip if all rotation axes are constrained
    if ((object->constraints & CONSTRAINTS_FREEZE_ROTATION_ALL) == CONSTRAINTS_FREEZE_ROTATION_ALL) {
        return;
    }

    // Transform angular impulse from world space to local space
    Quaternion rotation_inverse;
    quatConjugate(object->rotation, &rotation_inverse);
    Vector3 local_angular_impulse;
    quatMultVector(&rotation_inverse, angular_impulse, &local_angular_impulse);

    // Δω = I^-1 * angular_impulse (in local space)
    // Since we store diagonal inertia tensor, inverse is just 1/Ixx, 1/Iyy, 1/Izz
    Vector3 local_angular_velocity_change;
    local_angular_velocity_change.x = local_angular_impulse.x * object->_inv_local_intertia_tensor.x;
    local_angular_velocity_change.y = local_angular_impulse.y * object->_inv_local_intertia_tensor.y;
    local_angular_velocity_change.z = local_angular_impulse.z * object->_inv_local_intertia_tensor.z;

    // Apply per-axis constraints in LOCAL space
    if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_X) local_angular_velocity_change.x = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_Y) local_angular_velocity_change.y = 0.0f;
    if (object->constraints & CONSTRAINTS_FREEZE_ROTATION_Z) local_angular_velocity_change.z = 0.0f;

    // Transform angular velocity change back to world space
    Vector3 angular_velocity_change;
    quatMultVector(object->rotation, &local_angular_velocity_change, &angular_velocity_change);

    vector3Add(&object->angular_velocity, &angular_velocity_change, &object->angular_velocity);
}


void physics_object_apply_force_at_point(physics_object* object, Vector3* force, Vector3* world_point) {
    // Apply linear force
    vector3AddScaled(&object->acceleration, force, 1.0f / object->_mass, &object->acceleration);

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


void physics_object_set_angular_velocity(physics_object* object, Vector3* angular_velocity) {
    vector3Copy(angular_velocity, &object->angular_velocity);
}


contact* physics_object_nearest_contact(physics_object* object) {
    contact* nearest_target = NULL;
    contact* current = object->active_contacts;
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


bool physics_object_is_touching(physics_object* object, entity_id id) {
    contact* current = object->active_contacts;

    while (current) {
        if (current->other_object == id) {
            return true;
        }
            
        current = current->next;
    }

    return false;
}


void physics_object_set_mass(physics_object* object, float new_mass){
    assertf(new_mass > 0.0f, "Object Mass cannot be <= 0!");
    object->_mass = new_mass;
    object->_inv_mass = 1.0f / new_mass;
    // Calculate inertia tensor if inertia calculator is provided
    if (object->collision && object->collision->inertia_calculator) {
        object->collision->inertia_calculator(object, &object->_local_inertia_tensor);
    } else {
        // Fallback: treat as unit sphere if no calculator provided
        float default_inertia = 0.4f * new_mass;
        object->_local_inertia_tensor = (Vector3){{default_inertia, default_inertia, default_inertia}};
    }
    object->_inv_local_intertia_tensor.x = 1 / object->_local_inertia_tensor.x;
    object->_inv_local_intertia_tensor.y = 1 / object->_local_inertia_tensor.y;
    object->_inv_local_intertia_tensor.z = 1 / object->_local_inertia_tensor.z;
}


void physics_object_gjk_support_function(const void* data, const Vector3* direction, Vector3* output) {
    physics_object* object = (physics_object*)data;
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


void physics_object_recalculate_aabb(physics_object* object) {
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