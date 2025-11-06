#include "camera_controller.h"

#include "../time/time.h"
#include "../render/defs.h"

void camera_controller_update_position(struct camera_controller* controller, Transform* target) {
    Vector3 offset;

    //1. calc the forward vector according to inputs/player rotation
    if (joypad_get_buttons_held(0).z) {
        quatMultVector(&target->rotation, &gForward, &offset);
    } else {
        // Smooth the direction change
        Vector3 current_offset;
        vector3Sub(&target->position, &controller->camera->transform.position, &current_offset);
        current_offset.y = 0.0f;
        
        float mag = vector3Mag(&current_offset);
        if (mag > 0.1f) {
            vector3Scale(&current_offset, &offset, 1.0f / mag);
        } else {
            offset = gForward;
        }
    }


    
    //2. calculate the desired position of where the camera should be
    Vector3 desiredCamPos = {{target->position.x, target->position.y + CAMERA_FOLLOW_HEIGHT, target->position.z}};
    vector3AddScaled(&desiredCamPos, &offset, -CAMERA_FOLLOW_DISTANCE, &desiredCamPos);
    
    //3. raycast from camera Target to desired position
    raycast_hit ray_hit = (raycast_hit){0};
    Vector3 ray_origin = target->position;
    ray_origin.y += 0.5f; // Slight offset to avoid ground collision

    Vector3 ray_dir;
    vector3Sub(&desiredCamPos, &ray_origin, &ray_dir);
    float desired_distance = vector3Mag(&ray_dir);
    vector3NormalizeSelf(&ray_dir);
    
    raycast ray_to_cam = raycast_init(ray_origin, ray_dir, desired_distance, 
                                    RAYCAST_COLLISION_SCENE_MASK_STATIC_COLLISION, false, COLLISION_LAYER_TANGIBLE, COLLISION_LAYER_PLAYER); 
                                    
    raycast_cast(&ray_to_cam, &ray_hit);


    //4. Stable collision handling with hysteresis
    float target_distance;
    if (ray_hit.did_hit) {
        float hit_distance = ray_hit.distance - CAMERA_WALL_COLLISION_BUFFER;
        
        // Use hysteresis to prevent oscillation
        if (controller->collision_distance == 0.0f) {
            // First collision
            controller->collision_distance = hit_distance;
        } else {
            // Smooth transition with hysteresis
            float hysteresis_factor = 0.1f; // Adjust for more/less hysteresis
            
            if (hit_distance < controller->collision_distance - hysteresis_factor) {
                // Moving closer to wall - respond quickly
                controller->collision_distance = fm_lerp(controller->collision_distance, hit_distance, deltatime_sec * 8.0f);
            } else if (hit_distance > controller->collision_distance + hysteresis_factor) {
                // Moving away from wall - respond more slowly
                controller->collision_distance = fm_lerp(controller->collision_distance, hit_distance, deltatime_sec * 3.0f);
            }
            // Within hysteresis band - don't change
        }
        
        target_distance = fminf(desired_distance, controller->collision_distance);
    } else {
        // No collision - smoothly return to desired distance
        if (controller->collision_distance > 0.0f) {
            controller->collision_distance = fm_lerp(controller->collision_distance, desired_distance, deltatime_sec * 2.0f);
            if (fabsf(controller->collision_distance - desired_distance) < 0.01f) {
                controller->collision_distance = 0.0f;
            }
            target_distance = controller->collision_distance;
        } else {
            target_distance = desired_distance;
        }
    }
    
    // Calculate final position
    vector3AddScaled(&ray_origin, &ray_dir, target_distance, &desiredCamPos);
    desiredCamPos.y = fmaxf(ray_origin.y + 1, desiredCamPos.y);
    
    //5. Multi-stage smooth movement
    Vector3 current_pos = controller->camera->transform.position;
    Vector3 to_desired;
    vector3Sub(&desiredCamPos, &current_pos, &to_desired);
    float distance_to_desired = vector3Mag(&to_desired);
    
    // Variable lerp speed based on distance
    float lerp_speed;
    if (distance_to_desired > 6.0f) {
        lerp_speed = 7.0f; // Fast catch-up
    } else if (distance_to_desired > 1.0f) {
        lerp_speed = 4.0f; // Normal speed
    } else {
        lerp_speed = 2.0f; // Slow for precision
    }
    
    vector3Lerp(&current_pos, &desiredCamPos, deltatime_sec * lerp_speed, &controller->camera->transform.position);
    
    //6. Smooth look-at with damping
    Vector3 look_dir;
    vector3Sub(&target->position, &controller->camera->transform.position, &look_dir);
    
    // Add slight upward bias to avoid looking directly at feet
    look_dir.y += 0.2f;
    
    Quaternion desired_rotation;
    quatLook(&look_dir, &gUp, &desired_rotation);
    
    // Smooth rotation
    quatLerp(&controller->camera->transform.rotation, &desired_rotation, 
              deltatime_sec * 5.0f, &controller->camera->transform.rotation);
}

void camera_controller_update(struct camera_controller* controller) {
    vector3Lerp(&controller->target, &controller->player->transform.position, 1, &controller->target);
    camera_controller_update_position(controller, &controller->player->transform);
}

void camera_controller_init(struct camera_controller* controller, struct camera* camera, struct player* player) {
    controller->camera = camera;
    controller->player = player;
    //use fixed_update with less priority than UPDATE_PRIORITY_PLAYER to make camera controller update after player position is final
    fixed_update_add(controller, (update_callback)camera_controller_update, UPDATE_PRIORITY_CAMERA, UPDATE_LAYER_WORLD);

    controller->target = player->transform.position;
    controller->follow_distace = 3.0f;

    controller->camera->transform.position = player->transform.position;
    controller->camera->transform.scale = gOneVec;
    quatAxisAngle(&gRight, 0.0f, &controller->camera->transform.rotation);

    camera_controller_update_position(controller, &player->transform);
}

void camera_controller_destroy(struct camera_controller* controller) {
    update_remove(controller);
}