#include "sphere.h"

#include "../physics_object.h"
#include <math.h>

#define SQRT_1_3  0.577350269f

void sphere_support_function(const void* data, const Vector3* direction, Vector3* output) {
    physics_object* object = (physics_object*)data;

    float radius = object->collision->shape_data.sphere.radius;

    // For a sphere, the support point is along the direction at distance radius
    // support(d) = direction * radius
    // Note: direction is expected to be normalized

    output->x = direction->x * radius;
    output->y = direction->y * radius;
    output->z = direction->z * radius;
}

void sphere_bounding_box(const void* data, const Quaternion* rotation, AABB* box) {
    //rotation can be ignored for sphere
    physics_object* object = (physics_object*)data;
    const float r = object->collision->shape_data.sphere.radius;
    box->max = (Vector3){{r,r,r}};
    box->min = (Vector3){{-r,-r,-r}};
}

void sphere_inertia_tensor(void* data, Vector3* out) {
    physics_object* object = (physics_object*)data;
    float radius = object->collision->shape_data.sphere.radius;

    // Inertia tensor for a solid sphere: I = (2/5) * mass * r²
    // Same for all axes due to rotational symmetry
    float inertia = 0.4f * object->mass * radius * radius;

    out->x = inertia;
    out->y = inertia;
    out->z = inertia;
}