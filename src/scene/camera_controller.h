#ifndef __SCENE_CAMERA_CONTROLLER_H__
#define __SCENE_CAMERA_CONTROLLER_H__

#include "../render/camera.h"
#include "../player/player.h"
#include "../collision/raycast.h"

#define CAMERA_FOLLOW_DISTANCE  10.0f
#define CAMERA_FOLLOW_HEIGHT    5.5f

#define CAMERA_WALL_COLLISION_BUFFER 0.5f

struct camera_controller {
    struct camera* camera;
    struct player* player;
    float follow_distace;
    Vector3 target;
    float collision_distance;
};

void camera_controller_init(struct camera_controller* controller, struct camera* camera, struct player* player);

void camera_controller_destroy(struct camera_controller* controller);

#endif