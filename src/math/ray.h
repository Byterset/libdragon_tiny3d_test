#ifndef __RAY_H__
#define __RAY_H__

#include "vector3.h"
#include "transform.h"

typedef struct RayCast {
    Vector3 origin;
    Vector3 dir;
    float maxDistance;
} RayCast;

void rayTransform(Transform* transform, RayCast* ray, RayCast* output);
float rayDetermineDistance(RayCast* ray, Vector3* point);

#endif