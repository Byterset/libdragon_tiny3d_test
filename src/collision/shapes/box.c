#include "box.h"

#include "../physics_object.h"
#include <math.h>

void box_support_function(const void* data, const Vector3* direction, Vector3* output) {
    physics_object* object = (physics_object*)data;
    const Vector3 half_size = object->collision->shape_data.box.half_size;
    output->x = copysignf(half_size.x, direction->x);
    output->y = copysignf(half_size.y, direction->y);
    output->z = copysignf(half_size.z, direction->z);

}

void box_bounding_box(const void* data, const Quaternion* q, AABB* box) {
    physics_object* object = (physics_object*)data;
    Vector3* h = &object->collision->shape_data.box.half_size;

    float ex, ey, ez;

    if (!q) {
        ex = h->x;
        ey = h->y;
        ez = h->z;
    } else {
        // Precompute rotation matrix 3x3
        float x = q->x, y = q->y, z = q->z, w = q->w;
        float xx = x*x, yy = y*y, zz = z*z;
        float xy = x*y, xz = x*z, yz = y*z;
        float wx = w*x, wy = w*y, wz = w*z;

        float r00 = 1 - 2*(yy + zz), r01 = 2*(xy - wz), r02 = 2*(xz + wy);
        float r10 = 2*(xy + wz),     r11 = 1 - 2*(xx + zz), r12 = 2*(yz - wx);
        float r20 = 2*(xz - wy),     r21 = 2*(yz + wx),     r22 = 1 - 2*(xx + yy);

        // Compute extents using "box extents along rotated axes" formula
        ex = h->x*fabsf(r00) + h->y*fabsf(r01) + h->z*fabsf(r02);
        ey = h->x*fabsf(r10) + h->y*fabsf(r11) + h->z*fabsf(r12);
        ez = h->x*fabsf(r20) + h->y*fabsf(r21) + h->z*fabsf(r22);
    }

    box->min.x = -ex;
    box->min.y = -ey;
    box->min.z = -ez;
    box->max.x =  ex;
    box->max.y =  ey;
    box->max.z =  ez;
}

void box_inertia_tensor(void* data, Vector3* out) {
    physics_object* object = (physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;
    Vector3* half_size = &object->collision->shape_data.box.half_size;

    // Inertia tensor for a box with half-sizes (hx, hy, hz):
    // Ixx = (1/3) * mass * (hy² + hz²)
    // Iyy = (1/3) * mass * (hx² + hz²)
    // Izz = (1/3) * mass * (hx² + hy²)
    float hx_sq = half_size->x * half_size->x;
    float hy_sq = half_size->y * half_size->y;
    float hz_sq = half_size->z * half_size->z;

    float scale = object->mass / 3.0f;

    out->x = scale * (hy_sq + hz_sq);
    out->y = scale * (hx_sq + hz_sq);
    out->z = scale * (hx_sq + hy_sq);
}