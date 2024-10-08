#ifndef __RENDER_CAMERA_H__
#define __RENDER_CAMERA_H__

#include "../math/transform.h"
#include "../math/plane.h"
#include "../math/matrix.h"
#include <t3d/t3d.h>

struct ClippingPlanes {
    struct Plane planes[5];
};

struct camera {
    struct Transform transform;
    float fov;
    float near;
    float far;
};

void camera_init(struct camera* camera, float fov, float near, float far);

void camera_apply(struct camera* camera, T3DViewport* viewport, struct ClippingPlanes* clipping_planes, mat4x4 view_proj_matrix);

#endif