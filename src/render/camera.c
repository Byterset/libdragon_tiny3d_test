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
    t3d_viewport_set_projection(viewport, T3D_DEG_TO_RAD(camera->fov), camera->near, camera->far);
    Vector3 camTarget = cam_controller->target;

    camTarget.y += CAMERA_FOLLOW_HEIGHT;

    t3d_viewport_look_at(viewport, (T3DVec3*)&camera->transform.position, (T3DVec3*)&camTarget, (T3DVec3*)&gUp);
}