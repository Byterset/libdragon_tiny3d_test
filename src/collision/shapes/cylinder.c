#include "cylinder.h"

#include "../physics_object.h"
#include "../../math/matrix.h"
#include <math.h>
#include <libdragon.h>

#define SQRT_1_2 0.707106781f

void cylinder_support_function(const void* data, const Vector3* direction, Vector3* output) {
    struct physics_object* obj = (struct physics_object*)data;
    float r = obj->collision->shape_data.cylinder.radius;
    float h = obj->collision->shape_data.cylinder.half_height;

    float x = direction->x;
    float y = direction->y;
    float z = direction->z;

    // Y is always either +half_height or -half_height
    output->y = copysignf(h, y);

    // Fast radial approximation
    float abs_x = fabsf(x);
    float abs_z = fabsf(z);

    if (abs_x < SQRT_1_2 * (abs_x + abs_z) && abs_z < SQRT_1_2 * (abs_x + abs_z)) {
        // Approx corner
        output->x = (x >= 0.0f) ? r * SQRT_1_2 : -r * SQRT_1_2;
        output->z = (z >= 0.0f) ? r * SQRT_1_2 : -r * SQRT_1_2;
    } else if (abs_x > abs_z) {
        output->x = (x >= 0.0f) ? r : -r;
        output->z = 0.0f;
    } else {
        output->x = 0.0f;
        output->z = (z >= 0.0f) ? r : -r;
    }
}

void cylinder_bounding_box(const void* data, const Quaternion* q, AABB* box)
{
    struct physics_object* obj = (struct physics_object*)data;
    float h = obj->collision->shape_data.cylinder.half_height;
    float r = obj->collision->shape_data.cylinder.radius;

    // Local extents
    float ex = r;
    float ey = h;
    float ez = r;

    if (!q) {
        box->min = (Vector3){{ -ex, -ey, -ez }};
        box->max = (Vector3){{  ex,  ey,  ez }};
        return;
    }

    float x = q->x, y = q->y, z = q->z, w = q->w;

    // Precompute reused terms
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    // Rotation rows directly from quaternion
    float r00 = 1.0f - 2.0f*(yy + zz);
    float r01 =       2.0f*(xy - wz);
    float r02 =       2.0f*(xz + wy);

    float r10 =       2.0f*(xy + wz);
    float r11 = 1.0f - 2.0f*(xx + zz);
    float r12 =       2.0f*(yz - wx);

    float r20 =       2.0f*(xz - wy);
    float r21 =       2.0f*(yz + wx);
    float r22 = 1.0f - 2.0f*(xx + yy);

    // World extents = |R| * E
    float wxe =
        fabsf(r00) * ex +
        fabsf(r01) * ey +
        fabsf(r02) * ez;

    float wye =
        fabsf(r10) * ex +
        fabsf(r11) * ey +
        fabsf(r12) * ez;

    float wze =
        fabsf(r20) * ex +
        fabsf(r21) * ey +
        fabsf(r22) * ez;

    box->min = (Vector3){{ -wxe, -wye, -wze }};
    box->max = (Vector3){{  wxe,  wye,  wze }};
}

void cylinder_inertia_tensor(void* data, float mass, Vector3* out) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    float radius = shape_data->cylinder.radius;
    float half_height = shape_data->cylinder.half_height;
    float height = 2.0f * half_height;

    // Inertia tensor for a solid cylinder oriented along y-axis:
    // Ixx = Izz = (1/12) * mass * (3*r² + h²)  (perpendicular to cylinder axis)
    // Iyy = (1/2) * mass * r²  (along cylinder axis)
    float r_sq = radius * radius;
    float h_sq = height * height;

    float perpendicular_inertia = mass * (3.0f * r_sq + h_sq) / 12.0f;
    float axial_inertia = 0.5f * mass * r_sq;

    out->x = perpendicular_inertia;
    out->y = axial_inertia;
    out->z = perpendicular_inertia;
}