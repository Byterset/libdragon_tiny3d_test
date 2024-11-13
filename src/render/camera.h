#ifndef __RENDER_CAMERA_H__
#define __RENDER_CAMERA_H__

#include "../math/transform.h"
#include "../math/plane.h"
#include "../math/matrix.h"
#include "../scene/camera_controller.h"
#include <t3d/t3d.h>

struct ClippingPlanes {
    struct Plane planes[5];
};

struct camera {
    Transform transform;
    float fov;
    float near;
    float far;
};

void camera_init(struct camera* camera, float fov, float near, float far);

void camera_apply(struct camera* camera, T3DViewport* viewport, struct camera_controller* cam_controller);

#endif