#include "camera.h"
#include <math.h>
#include "defs.h"

void camera_init(struct camera* camera, float fov, float near, float far) {
    transformInitIdentity(&camera->transform);
    camera->fov = fov;
    camera->near = near;
    camera->far = far;
}

void camera_apply(struct camera* camera, T3DViewport* viewport, struct camera_controller* cam_controller) {
    t3d_viewport_set_projection(viewport, T3D_DEG_TO_RAD(camera->fov), camera->near * SCENE_SCALE, camera->far * SCENE_SCALE);
    Vector3 camPos, camTarget;
    vector3Scale(&camera->transform.position, &camPos, SCENE_SCALE);
    vector3Scale(&cam_controller->target, &camTarget, SCENE_SCALE);
    camTarget.y += CAMERA_FOLLOW_HEIGHT * SCENE_SCALE;

    t3d_viewport_look_at(viewport, (T3DVec3*)&camPos, (T3DVec3*)&camTarget, (T3DVec3*)&gUp);
}