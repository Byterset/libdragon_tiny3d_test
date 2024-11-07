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
    T3DVec3 camPos, camTarget;
    camPos.v[0] = camera->transform.position.x * SCENE_SCALE;
    camPos.v[1] = (camera->transform.position.y) * SCENE_SCALE;
    camPos.v[2] = camera->transform.position.z * SCENE_SCALE;

    camTarget.v[0] = cam_controller->target.x * SCENE_SCALE;
    camTarget.v[1] = (cam_controller->target.y + CAMERA_FOLLOW_HEIGHT) * SCENE_SCALE;
    camTarget.v[2] = cam_controller->target.z * SCENE_SCALE;
    t3d_viewport_look_at(viewport, &camPos, &camTarget, &(T3DVec3){{0, 1, 0}});
}