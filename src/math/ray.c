#include "ray.h"
#include "../collision/collision_scene.h"

RayCast RayCast_create(Vector3 origin, Vector3 dir, float maxDistance){
    RayCast ray = {
        .origin = origin,
        .dir = dir,
        .maxDistance = maxDistance
    };
    ray._invDir.x = safeInvert(dir.x);
    ray._invDir.y = safeInvert(dir.y);
    ray._invDir.z = safeInvert(dir.z);
    return ray;
}

void RayCast_transform(Transform* transform, RayCast* ray, RayCast* output) {
    transformPoint(transform, &ray->origin, &output->origin);
    quatMultVector(&transform->rotation, &ray->dir, &output->dir);
}

float RayCast_calc_distance(RayCast* ray, Vector3* point) {
    Vector3 relative;
    vector3Sub(point, &ray->origin, &relative);
    return vector3Dot(&relative, &ray->dir);
}


