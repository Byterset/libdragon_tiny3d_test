#include "physics_object.h"
#include <assert.h>
#include "../time/time.h"
#include "../math/minmax.h"
#include <math.h>
#include <stddef.h>

void physics_object_init(
    entity_id entity_id,
    struct physics_object* object, 
    struct physics_object_collision_data* type,
    uint16_t collision_layers,
    struct Vector3* position, 
    struct Quaternion* rotation,
    float mass
) {
    assert(mass > 0.0f);
    object->entity_id = entity_id;
    object->collision = type;
    object->position = position;
    object->verlet_prev_position = *position;
    object->prev_step_pos = *position;
    object->rotation = rotation;
    object->velocity = gZeroVec;
    object->center_offset = gZeroVec;
    object->time_scalar = 1.0f;
    object->mass = mass;
    object->mass_inv = 1.0f / mass;
    object->has_gravity = 1;
    object->is_trigger = 0;
    object->is_fixed = 0;
    object->is_out_of_bounds = 0;
    object->is_grounded = 0;
    object->collision_layers = collision_layers;
    object->collision_group = 0;
    object->active_contacts = 0;
    physics_object_recalculate_aabb(object);
}

/// @brief Applies gravity and performs verlet integration on the object.
/// Will not do so if the object is a trigger or fixed
/// @param object 
void physics_object_update(struct physics_object* object) {
    if (object->is_trigger | object->is_fixed) {
        return;
    }

    if (object->has_gravity) {
        object->acceleration.y += GRAVITY_CONSTANT;
    }
    object->is_grounded = 0;

    //new_pos = current_pos + (current_pos − previous_pos) + a * (Δt^2)
    struct Vector3 displacement;
    vector3Sub(object->position, &object->verlet_prev_position, &displacement); // current_pos − previous_pos
    vector3Scale(&object->acceleration, &object->acceleration, FIXED_DELTATIME_SQUARED); // a * (Δt^2)
    vector3Scale(&displacement, &displacement, 0.995f); // introduce damping
    vector3Add(&displacement, &object->acceleration, &displacement); // current_pos − previous_pos + a * (Δt^2)
    vector3Copy(object->position, &object->verlet_prev_position);
    vector3Add(object->position, &displacement, object->position); // add to current position
    object->velocity = physics_object_get_velocity(object); // update the inferred velocity
    object->acceleration = gZeroVec; // reset forces
}

void physics_object_apply_constraints(struct physics_object* object){
    if (object->position->y <= -20){
        physics_object_position_no_force(object, &(struct Vector3){object->position->x, -20, object->position->z});
    }
    if (object->position->y >= 2000){
        physics_object_position_no_force(object, &(struct Vector3){object->position->x, 2000, object->position->z});
    }
    if (object->position->x <= -2000){
        physics_object_position_no_force(object, &(struct Vector3){-2000, object->position->y, object->position->z});
    }
    if (object->position->x >= 2000){
        physics_object_position_no_force(object, &(struct Vector3){2000, object->position->y, object->position->z});
    }
    if (object->position->z <= -2000){
        physics_object_position_no_force(object, &(struct Vector3){object->position->x, object->position->y, -2000});
    }
    if (object->position->z >= 2000){
        physics_object_position_no_force(object, &(struct Vector3){object->position->x, object->position->y, 2000});
    }
}

/// @brief Accelerates the object by the given acceleration vector. 
/// 
/// Keep in mind that the acceleration is applied for one Physics update cycle and should be scaled by the 
/// PHYSICS_TICKRATE so it scales to one second instead.
/// @param object 
/// @param acceleration 
void physics_object_accelerate(struct physics_object* object, struct Vector3* acceleration) {
    vector3Add(&object->acceleration, acceleration, &object->acceleration);
}

void physics_object_translate_no_force(struct physics_object* object, struct Vector3* translation) {
    vector3Add(object->position, translation, object->position);
    vector3Add(&object->verlet_prev_position, translation, &object->verlet_prev_position);
}

void physics_object_position_no_force(struct physics_object* object, struct Vector3* position) {
    vector3Copy(position, object->position);
    vector3Copy(position, &object->verlet_prev_position);
}

struct Vector3 physics_object_get_velocity(struct physics_object* object){
	struct Vector3 vel;
	vector3Copy(object->position, &vel);
    vector3Sub(&vel, &object->verlet_prev_position, &vel);
    return vel;
}

void physics_object_set_velocity(struct physics_object* object, struct Vector3* velocity){
    vector3Copy(object->position, &object->verlet_prev_position);
    vector3Sub(&object->verlet_prev_position, velocity, &object->verlet_prev_position);
}

void physics_object_apply_impulse(struct physics_object* object, struct Vector3* impulse) {
    vector3Sub(&object->verlet_prev_position, impulse, &object->verlet_prev_position);
}



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

void physics_object_gjk_support_function(void* data, struct Vector3* direction, struct Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    struct Vector3 world_center;
    struct Vector3 localDir;
    if(object->rotation){
        struct Quaternion inv_rotation;
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
    struct Vector3 offset;

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