#include "dynamic_object.h"
#include <assert.h>
#include "../time/time.h"
#include "../math/minmax.h"
#include <math.h>
#include <stddef.h>

void dynamic_object_init(
    entity_id entity_id,
    struct dynamic_object* object, 
    struct dynamic_object_type* type,
    uint16_t collision_layers,
    struct Vector3* position, 
    struct Vector2* rotation,
    float mass
) {
    assert(mass > 0.0f);
    object->entity_id = entity_id;
    object->type = type;
    object->position = position;
    object->prev_position = *position;
    object->rotation = rotation;
    object->pitch = 0;
    object->velocity = gZeroVec;
    object->center = gZeroVec;
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
    dynamic_object_recalc_bb(object);
}

/// @brief Applies gravity and performs verlet integration on the object.
/// Will not do so if the object is a trigger or fixed
/// @param object 
void dynamic_object_update(struct dynamic_object* object) {
    if (object->is_trigger | object->is_fixed) {
        return;
    }

    if (object->has_gravity) {
        object->acceleration.y += GRAVITY_CONSTANT;
    }
    object->is_grounded = 0;

    //new_pos = current_pos + (current_pos − previous_pos) + a * (Δt^2)
    struct Vector3 displacement;
    vector3Sub(object->position, &object->prev_position, &displacement); // current_pos − previous_pos
    vector3Scale(&object->acceleration, &object->acceleration, FIXED_DELTATIME_SQUARED); // a * (Δt^2)
    vector3Scale(&displacement, &displacement, 0.995f); // introduce damping
    vector3Add(&displacement, &object->acceleration, &displacement); // current_pos − previous_pos + a * (Δt^2)
    vector3Copy(object->position, &object->prev_position);
    vector3Add(object->position, &displacement, object->position); // add to current position
    object->velocity = dynamic_object_get_velocity(object); // update the inferred velocity
    object->acceleration = gZeroVec; // reset forces
}

void dynamic_object_apply_constraints(struct dynamic_object* object){
    if (object->position->y <= 0){
        dynamic_object_position_no_force(object, &(struct Vector3){object->position->x, 0, object->position->z});
        object->is_grounded = 1;
    }
    if (object->position->y >= 2000){
        dynamic_object_position_no_force(object, &(struct Vector3){object->position->x, 2000, object->position->z});
    }
    if (object->position->x <= -2000){
        dynamic_object_position_no_force(object, &(struct Vector3){-2000, object->position->y, object->position->z});
    }
    if (object->position->x >= 2000){
        dynamic_object_position_no_force(object, &(struct Vector3){2000, object->position->y, object->position->z});
    }
    if (object->position->z <= -2000){
        dynamic_object_position_no_force(object, &(struct Vector3){object->position->x, object->position->y, -2000});
    }
    if (object->position->z >= 2000){
        dynamic_object_position_no_force(object, &(struct Vector3){object->position->x, object->position->y, 2000});
    }
}

/// @brief Accelerates the object by the given acceleration vector. 
/// 
/// Keep in mind that the acceleration is applied for one Physics update cycle and should be scaled by the 
/// PHYSICS_TICKRATE so it scales to one second instead.
/// @param object 
/// @param acceleration 
void dynamic_object_accelerate(struct dynamic_object* object, struct Vector3* acceleration) {
    vector3Add(&object->acceleration, acceleration, &object->acceleration);
}

void dynamic_object_translate_no_force(struct dynamic_object* object, struct Vector3* translation) {
    vector3Add(object->position, translation, object->position);
    vector3Add(&object->prev_position, translation, &object->prev_position);
}

void dynamic_object_position_no_force(struct dynamic_object* object, struct Vector3* position) {
    vector3Copy(position, object->position);
    vector3Copy(position, &object->prev_position);
}

struct Vector3 dynamic_object_get_velocity(struct dynamic_object* object){
	struct Vector3 vel;
	vector3Copy(object->position, &vel);
    vector3Sub(&vel, &object->prev_position, &vel);
    return vel;
}

void dynamic_object_set_velocity(struct dynamic_object* object, struct Vector3* velocity){
    vector3Copy(object->position, &object->prev_position);
    vector3Sub(&object->prev_position, velocity, &object->prev_position);
}

void dynamic_object_apply_impulse(struct dynamic_object* object, struct Vector3* impulse) {
    vector3Sub(&object->prev_position, impulse, &object->prev_position);
}



struct contact* dynamic_object_nearest_contact(struct dynamic_object* object) {
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

bool dynamic_object_is_touching(struct dynamic_object* object, entity_id id) {
    struct contact* current = object->active_contacts;

    while (current) {
        if (current->other_object == id) {
            return true;
        }
            
        current = current->next;
    }

    return false;
}

void dynamic_object_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct dynamic_object* object = (struct dynamic_object*)data;

    struct Vector3 rotated_dir;

    if (!object->rotation && !object->pitch) {
        object->type->minkowsi_sum(&object->type->data, direction, output);
        vector3Add(output, object->position, output);
        vector3Add(output, &object->center, output);
        return;
    }

    struct Vector3 pitched_dir;

    if (object->pitch) {
        pitched_dir.x = direction->x;
        pitched_dir.y = direction->y * object->pitch->x - direction->z * object->pitch->y;
        pitched_dir.z = direction->z * object->pitch->x + direction->y * object->pitch->y;
    } else {
        pitched_dir = *direction;
    }

    rotated_dir.x = pitched_dir.x * object->rotation->x + pitched_dir.z * object->rotation->y;
    rotated_dir.y = pitched_dir.y;
    rotated_dir.z = pitched_dir.z * object->rotation->x - pitched_dir.x * object->rotation->y;

    struct Vector3 unrotated_out;
    
    object->type->minkowsi_sum(&object->type->data, &rotated_dir, &unrotated_out);

    struct Vector3 unpitched_out;

    unpitched_out.x = unrotated_out.x * object->rotation->x - unrotated_out.z * object->rotation->y + object->position->x;
    unpitched_out.y = unrotated_out.y + object->position->y;
    unpitched_out.z = unrotated_out.z * object->rotation->x + unrotated_out.x * object->rotation->y + object->position->z;

    if (object->pitch) {
        output->x = unpitched_out.x;
        output->y = unpitched_out.y * object->pitch->x + unpitched_out.z * object->pitch->y;
        output->z = unpitched_out.z * object->pitch->x - unpitched_out.y * object->pitch->y;
    } else {
        *output = unpitched_out;
    }

    vector3Add(output, &object->center, output);
}

/// @brief re-caluclates the bounding box of the object using the collision type's bounding box function
/// @param object 
void dynamic_object_recalc_bb(struct dynamic_object* object) {
    object->type->bounding_box(&object->type->data, object->rotation, &object->bounding_box);
    struct Vector3 offset;
    vector3Add(&object->center, object->position, &offset);
    vector3Add(&object->bounding_box.min, &offset, &object->bounding_box.min);
    vector3Add(&object->bounding_box.max, &offset, &object->bounding_box.max);
}