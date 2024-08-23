#include "camera_controller.h"

#include "../time/time.h"
#include "../render/defs.h"

#define CAMERA_FOLLOW_DISTANCE  45.0f
#define CAMERA_FOLLOW_HEIGHT    30.0f

void camera_controller_update_position(struct camera_controller* controller, struct Transform* target) {
    struct Vector3 offset;

    if (joypad_get_buttons_held(0).z) {
        quatMultVector(&target->rotation, &gForward, &offset);
    } else {
        vector3Sub(&target->position, &controller->camera->transform.position, &offset);

        offset.y = 0.0f;
        vector3Normalize(&offset, &offset);

        if (vector3MagSqrd(&offset) < 0.1f) {
            offset = gForward;
        }
    }
    

    struct Vector3 targetPosition;
    vector3AddScaled(&target->position, &offset, -CAMERA_FOLLOW_DISTANCE, &targetPosition);
    targetPosition.y += CAMERA_FOLLOW_HEIGHT;

    vector3Lerp(&controller->camera->transform.position, &targetPosition, delta_time_s * 6.0f, &controller->camera->transform.position);

    vector3Sub(&target->position, &controller->camera->transform.position, &offset);
    offset.y += CAMERA_FOLLOW_HEIGHT;
    quatLook(&offset, &gUp, &controller->camera->transform.rotation);
}

void camera_controller_update(struct camera_controller* controller) {
    vector3Lerp(&controller->target, &controller->player->transform.position, delta_time_s * 54.0f, &controller->target);

    camera_controller_update_position(controller, &controller->player->transform);
}

void camera_controller_init(struct camera_controller* controller, struct camera* camera, struct player* player) {
    controller->camera = camera;
    controller->player = player;

    update_add(controller, (update_callback)camera_controller_update, UPDATE_PRIORITY_CAMERA, UPDATE_LAYER_WORLD);

    controller->target = player->transform.position;
    controller->follow_distace = 25.0f;

    controller->camera->transform.position = gZeroVec;
    controller->camera->transform.scale = gOneVec;
    quatAxisAngle(&gRight, 0.0f, &controller->camera->transform.rotation);

    camera_controller_update_position(controller, &player->transform);
}

void camera_controller_destroy(struct camera_controller* controller) {
    update_remove(controller);
}