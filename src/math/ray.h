#ifndef __RAY_H__
#define __RAY_H__

#include "vector3.h"
#include "transform.h"

struct RayCast {
    Vector3 origin;
    Vector3 dir;
    float maxDistance;
};

void rayTransform(Transform* transform, struct RayCast* ray, struct RayCast* output);
float rayDetermineDistance(struct RayCast* ray, Vector3* point);

#endif