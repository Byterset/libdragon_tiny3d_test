#include "pyramid.h"

#include "../physics_object.h"
#include <math.h>

void pyramid_support_function(const void* data, const Vector3* direction, Vector3* output) {
    physics_object* object = (physics_object*)data;
    float hwx = object->collision->shape_data.pyramid.base_half_widths.x;
    float hwz = object->collision->shape_data.pyramid.base_half_widths.y;
    float hh = object->collision->shape_data.pyramid.half_height;

    // Apex at (0, +hh, 0)
    float apex_dot = hh * direction->y;
    // Base at (+-hwx, -hh, +-hwz)
    float base_dot = fabsf(direction->x) * hwx + fabsf(direction->z) * hwz - hh * direction->y;

    if(apex_dot > base_dot){
        output->x = 0.0f;
        output->y = hh;
        output->z = 0.0f;
    }
    else{
        output->x = copysignf(hwx, direction->x);
        output->y = -hh;
        output->z = copysignf(hwz, direction->z);
    }
}

void pyramid_bounding_box(const void* data, const Quaternion* q, AABB* box) {
    physics_object* object = (physics_object*)data;
    const float hh = object->collision->shape_data.pyramid.half_height;
    const float hwx = object->collision->shape_data.pyramid.base_half_widths.x;
    const float hwz = object->collision->shape_data.pyramid.base_half_widths.y;

    float ex, ey, ez;

    if (!q) {
        ex = hwx;
        ey = hh;
        ez = hwz;
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
        ex = hwx*fabsf(r00) + hh*fabsf(r01) + hwz*fabsf(r02);
        ey = hwx*fabsf(r10) + hh*fabsf(r11) + hwz*fabsf(r12);
        ez = hwx*fabsf(r20) + hh*fabsf(r21) + hwz*fabsf(r22);
    }

    box->min.x = -ex;
    box->min.y = -ey;
    box->min.z = -ez;
    box->max.x =  ex;
    box->max.y =  ey;
    box->max.z =  ez;
}

void pyramid_inertia_tensor(void* data, Vector3* out) {
    physics_object* object = (physics_object*)data;
    const float hh = object->collision->shape_data.pyramid.half_height;
    const float hwx = object->collision->shape_data.pyramid.base_half_widths.x;
    const float hwz = object->collision->shape_data.pyramid.base_half_widths.y;

    // With the apex at (0, hh, 0) and the base at (0, -hh, 0), 
    // the total height of the pyramid is 2 * hh. The center of mass is located at (0, -0.5 * hh, 0)

    // Ixx = M *(hwz^2 / 5 + 3*hh^2 / 20)
    // Ixx = M *(hwx^2 / 5 + hwz^2 / 5)
    // Izz = M *(hwx^2 / 5 + 3*hh^2 / 20)

    float m_div_20 = object->_mass * 0.05f;
    float m_div_5 = object->_mass * 0.2f;

    // Rectangulat pyramid inertia about centroid:
    float Ixx = (m_div_5 * hwz * hwz) + (m_div_20 * 3 * hh * hh);
    float Iyy = (m_div_5 * hwx * hwx) + (m_div_5 * hwz * hwz);
    float Izz = (m_div_5 * hwx * hwx) + (m_div_20 * 3 * hh * hh);

    out->x = Ixx;
    out->y = Iyy;
    out->z = Izz;
}