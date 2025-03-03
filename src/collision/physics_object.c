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
/// @param mass 
void physics_object_init(
    entity_id entity_id,
    struct physics_object* object, 
    struct physics_object_collision_data* collision,
    uint16_t collision_layers,
    Vector3* position, 
    Quaternion* rotation,
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
    object->center_offset = gZeroVec;
    object->time_scalar = 1.0f;
    object->gravity_scalar = 1.0f;
    object->mass = mass;
    object->has_gravity = 1;
    object->is_trigger = 0;
    object->is_fixed = 0;
    object->is_out_of_bounds = 0;
    object->is_grounded = 0;
    object->is_sleeping = 0;
    object->collision_layers = collision_layers;
    object->collision_group = 0;
    object->active_contacts = 0;
    physics_object_recalculate_aabb(object);
}

void physics_object_update_euler(struct physics_object* object) {
    if (object->is_trigger | object->is_fixed) {
        return;
    }

    if (object->has_gravity) {
        object->velocity.y += FIXED_DELTATIME * object->time_scalar * GRAVITY_CONSTANT * object->gravity_scalar;
    }
    vector3AddScaled(&object->velocity, &object->acceleration, FIXED_DELTATIME * object->time_scalar, &object->velocity);
    object->velocity.y = clampf(object->velocity.y, -PHYS_OBJECT_TERMINAL_Y_VELOCITY, PHYS_OBJECT_TERMINAL_Y_VELOCITY);

    object->is_grounded = 0;

    vector3AddScaled(object->position, &object->velocity, FIXED_DELTATIME * object->time_scalar, object->position);
    object->acceleration = gZeroVec;

}

void physics_object_update_velocity_verlet_simple(struct physics_object* object) {
    if (object->is_trigger | object->is_fixed) {
        return;
    }
    vector3AddScaled(&object->velocity, &object->acceleration, FIXED_DELTATIME * object->time_scalar, &object->velocity);
    object->velocity.y = clampf(object->velocity.y, -PHYS_OBJECT_TERMINAL_Y_VELOCITY, PHYS_OBJECT_TERMINAL_Y_VELOCITY);

    object->is_grounded = 0;

    vector3AddScaled(object->position, &object->velocity, FIXED_DELTATIME * object->time_scalar, object->position);
    vector3AddScaled(object->position, &object->acceleration, FIXED_DELTATIME * FIXED_DELTATIME * object->time_scalar * 0.5f, object->position);
    object->acceleration = gZeroVec;

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