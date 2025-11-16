#include "cone.h"

#include "../physics_object.h"
#include "../../math/minmax.h"
#include "../../math/mathf.h"
#include <math.h>

void cone_support_function(const void* data, const Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = (union physics_object_collision_shape_data*)&object->collision->shape_data;

    const float r = shape_data->cone.radius; // radius ρ
    const float h = shape_data->cone.half_height; // half height η

    const float dx = direction->x;
    const float dy = direction->y;
    const float dz = direction->z;

    const float sin_alpha = r / sqrtf((r*r) + 4*h*h);   // sin(α) = ρ / √(ρ² + (2 * η)²)   top angle α
    const float sin2 = sin_alpha * sin_alpha;
    const float sigma2 = dx*dx + dz*dz;               // squared radial length σ = √(x²+z²)
    const float dy2 = dy*dy;
    const float d2 = dy2 + sigma2; //squared length of dir
    // Case 1: choose apex if y > ‖d‖ * sin(α)
    if (dy > 0 && dy2 > (d2 * sin2)) {
        output->x = 0.0f;
        output->y = h;
        output->z = 0.0f;
        return;
    }

    // Case 2: choose base circle support (if radial direction exists)
    if (sigma2 > 0.0f) {
        float invSigma = 1.0f / sqrtf(sigma2);
        output->x = r * dx * invSigma;
        output->y = -h;
        output->z = r * dz * invSigma;
        return;
    }

    // Case 3: direction is purely vertical → support is straight down
    output->x = 0.0f;
    output->y = -h;
    output->z = 0.0f;
}

void cone_bounding_box(const void* data, const Quaternion* q, AABB* box)
{
    struct physics_object* object = (struct physics_object*)data;
    float h = object->collision->shape_data.cone.half_height;
    float r = object->collision->shape_data.cone.radius;

    // Local points
    Vector3 apex = {{0,  h, 0}};
    Vector3 base[4] = {
        {{ r, -h,  0 }},
        {{-r, -h,  0 }},
        {{ 0, -h,  r }},
        {{ 0, -h, -r }},
    };

    if (!q) {
        // No rotation → trivial AABB
        box->min = (Vector3){{ -r, -h, -r }};
        box->max = (Vector3){{  r,  h,  r }};
        return;
    }

    // --- Precompute rotation matrix from quaternion ---
    float x = q->x, y = q->y, z = q->z, w = q->w;
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    float r00 = 1 - 2*(yy + zz), r01 = 2*(xy - wz), r02 = 2*(xz + wy);
    float r10 = 2*(xy + wz),     r11 = 1 - 2*(xx + zz), r12 = 2*(yz - wx);
    float r20 = 2*(xz - wy),     r21 = 2*(yz + wx),     r22 = 1 - 2*(xx + yy);

    // Function to multiply vector by rotation matrix
    #define ROTATE(v, out)            \
        do {                          \
            out.x = r00*v.x + r01*v.y + r02*v.z; \
            out.y = r10*v.x + r11*v.y + r12*v.z; \
            out.z = r20*v.x + r21*v.y + r22*v.z; \
        } while(0)

    Vector3 r_apex;
    ROTATE(apex, r_apex);

    Vector3 bmin = r_apex;
    Vector3 bmax = r_apex;

    for (int i = 0; i < 4; i++) {
        Vector3 p;
        ROTATE(base[i], p);
        vector3Min(&bmin, &p, &bmin);
        vector3Max(&bmax, &p, &bmax);
    }

    box->min = bmin;
    box->max = bmax;
}

void cone_inertia_tensor(void* data, float mass, Vector3* out) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    float radius = shape_data->cone.radius;
    float half_height = shape_data->cone.half_height;
    float height = 2.0f * half_height;

    // Inertia tensor for a solid cone oriented along y-axis:
    // Ixx = Izz = (3/80) * mass * (4*r² + h²)  (perpendicular to cone axis)
    // Iyy = (3/10) * mass * r²  (along cone axis)
    float r_sq = radius * radius;
    float h_sq = height * height;

    float perpendicular_inertia = (3.0f / 80.0f) * mass * (4.0f * r_sq + h_sq);
    float axial_inertia = 0.3f * mass * r_sq;

    out->x = perpendicular_inertia;
    out->y = axial_inertia;
    out->z = perpendicular_inertia;
}