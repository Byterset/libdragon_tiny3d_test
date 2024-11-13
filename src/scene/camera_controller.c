#include "camera_controller.h"

#include "../time/time.h"
#include "../render/defs.h"

void camera_controller_update_position(struct camera_controller* controller, Transform* target) {
    Vector3 offset;

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


    
    //calculate the target position of where the camera should be
    Vector3 targetPosition;
    vector3AddScaled(&target->position, &offset, -CAMERA_FOLLOW_DISTANCE, &targetPosition);
    targetPosition.y += CAMERA_FOLLOW_HEIGHT;

    // move the target Position up or down to look from above or below
    //TODO: this has been added to test the skybox code and can be removed safely later
    if (joypad_get_buttons_held(0).d_down)
    {
        targetPosition.y -= 15;
    }
    else if (joypad_get_buttons_held(0).d_up)
    {
        targetPosition.y += 15;
    }

    //move the camera towards the target position
    vector3Lerp(&controller->camera->transform.position, &targetPosition, frametime_sec * 6.0f, &controller->camera->transform.position);

    //look at the target
    vector3Sub(&target->position, &controller->camera->transform.position, &offset);
    offset.y += CAMERA_FOLLOW_HEIGHT;
    quatLook(&offset, &gUp, &controller->camera->transform.rotation);
}

void camera_controller_update(struct camera_controller* controller) {
    vector3Lerp(&controller->target, &controller->player->transform.position, 1, &controller->target);
    camera_controller_update_position(controller, &controller->player->transform);
}

void camera_controller_init(struct camera_controller* controller, struct camera* camera, struct player* player) {
    controller->camera = camera;
    controller->player = player;

    update_add(controller, (update_callback)camera_controller_update, UPDATE_PRIORITY_CAMERA, UPDATE_LAYER_WORLD);

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