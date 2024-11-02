#include "ray.h"

void rayTransform(struct Transform* transform, struct RayCast* ray, struct RayCast* output) {
    transformPoint(transform, &ray->origin, &output->origin);
    quatMultVector(&transform->rotation, &ray->dir, &output->dir);
}

float rayDetermineDistance(struct RayCast* ray, struct Vector3* point) {
    struct Vector3 relative;
    vector3Sub(point, &ray->origin, &relative);
    return vector3Dot(&relative, &ray->dir);
}
