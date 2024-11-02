#ifndef __RAY_H__
#define __RAY_H__

#include "vector3.h"
#include "transform.h"

struct RayCast {
    struct Vector3 origin;
    struct Vector3 dir;
    float maxDistance;
};

void rayTransform(struct Transform* transform, struct RayCast* ray, struct RayCast* output);
float rayDetermineDistance(struct RayCast* ray, struct Vector3* point);

#endif