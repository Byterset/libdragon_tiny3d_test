#include "ray.h"

void rayTransform(Transform* transform, RayCast* ray, RayCast* output) {
    transformPoint(transform, &ray->origin, &output->origin);
    quatMultVector(&transform->rotation, &ray->dir, &output->dir);
}

float rayDetermineDistance(RayCast* ray, Vector3* point) {
    Vector3 relative;
    vector3Sub(point, &ray->origin, &relative);
    return vector3Dot(&relative, &ray->dir);
}
