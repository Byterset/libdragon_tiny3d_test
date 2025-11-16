#include "capsule.h"

#include "../physics_object.h"
#include <math.h>
#include "sphere.h"

void capsule_support_function(const void* data, const Vector3* direction, Vector3* output) {
    struct physics_object* object = (struct physics_object*)data;
    float half_h = object->collision->shape_data.capsule.inner_half_height;
    float radius = object->collision->shape_data.capsule.radius;

    // Determine the Y-coordinate along the capsule axis
    float y = copysignf(half_h, direction->y);

    // Add radius in the direction of the vector
    output->x = direction->x * radius;
    output->y = direction->y * radius + y;
    output->z = direction->z * radius;
}


void capsule_bounding_box(const void* data, const Quaternion* q, AABB* box) {
    struct physics_object* object = (struct physics_object*)data;
    float half_h = object->collision->shape_data.capsule.inner_half_height;
    float radius = object->collision->shape_data.capsule.radius;

    // Local Y-axis endpoint
    Vector3 endpoint = {{0, half_h, 0}};
    Vector3 r;

    if (!q) {
        r = endpoint;
    } else {
        // Precompute rotation matrix from quaternion
        float x = q->x, y = q->y, z = q->z, w = q->w;
        float xx = x*x, yy = y*y, zz = z*z;
        float xy = x*y, xz = x*z, yz = y*z;
        float wx = w*x, wy = w*y, wz = w*z;

        float r00 = 1 - 2*(yy + zz), r01 = 2*(xy - wz), r02 = 2*(xz + wy);
        float r10 = 2*(xy + wz),     r11 = 1 - 2*(xx + zz), r12 = 2*(yz - wx);
        float r20 = 2*(xz - wy),     r21 = 2*(yz + wx),     r22 = 1 - 2*(xx + yy);

        // Rotate endpoint
        r.x = r00*endpoint.x + r01*endpoint.y + r02*endpoint.z;
        r.y = r10*endpoint.x + r11*endpoint.y + r12*endpoint.z;
        r.z = r20*endpoint.x + r21*endpoint.y + r22*endpoint.z;
    }

    const float r_abs_x = fabsf(r.x);
    const float r_abs_y = fabsf(r.y);
    const float r_abs_z = fabsf(r.z);

    // Symmetry: the other endpoint is just -r
    // AABB min/max are branchless using fabs
    box->min.x = -r_abs_x - radius;
    box->max.x =  r_abs_x + radius;
    box->min.y = -r_abs_y - radius;
    box->max.y =  r_abs_y + radius;
    box->min.z = -r_abs_z - radius;
    box->max.z =  r_abs_z + radius;
}


void capsule_inertia_tensor(void* data, float mass, Vector3* out) {
    struct physics_object* object = (struct physics_object*)data;
    union physics_object_collision_shape_data* shape_data = &object->collision->shape_data;

    float radius = shape_data->capsule.radius;
    float inner_half_height = shape_data->capsule.inner_half_height;
    float cylinder_height = 2.0f * inner_half_height;

    // Calculate volumes to distribute mass

    float cylinder_volume = PI * radius * radius * cylinder_height;
    float sphere_volume = (4.0f / 3.0f) * PI * radius * radius * radius;
    float total_volume = cylinder_volume + sphere_volume;

    float cylinder_mass = mass * (cylinder_volume / total_volume);
    float sphere_mass = mass * (sphere_volume / total_volume);

    // Cylinder inertia (oriented along y-axis)
    float r_sq = radius * radius;
    float h_sq = cylinder_height * cylinder_height;
    float cyl_perp = cylinder_mass * (3.0f * r_sq + h_sq) / 12.0f;
    float cyl_axial = 0.5f * cylinder_mass * r_sq;

    // Sphere inertia (centered at origin, but the hemispheres are offset)
    float sphere_inertia = 0.4f * sphere_mass * r_sq;

    // Parallel axis theorem for the two hemisphere caps offset by ±inner_half_height
    // I_parallel = I_cm + m * d² where d is the distance from center of mass
    // Each hemisphere has mass sphere_mass/2 and is offset by inner_half_height
    float offset_sq = inner_half_height * inner_half_height;
    float hemisphere_mass = sphere_mass * 0.5f;
    float sphere_perp = sphere_inertia + 2.0f * hemisphere_mass * offset_sq;

    // Combine cylinder and sphere contributions
    out->x = cyl_perp + sphere_perp;
    out->y = cyl_axial + sphere_inertia;
    out->z = cyl_perp + sphere_perp;
}
