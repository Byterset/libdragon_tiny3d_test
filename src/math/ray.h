#ifndef __RAY_H__
#define __RAY_H__

#include "vector3.h"
#include "transform.h"
#include "mathf.h"

/// @brief The RayCast structure represents a RayCast in 3D space. Don't create this structure directly, use RayCast_create instead.
typedef struct RayCast {
    Vector3 origin;
    Vector3 dir;
    Vector3 _invDir;
    float maxDistance;
} RayCast;

typedef struct RayCastHit {
    Vector3 point;
    Vector3 normal;
    float distance;
} RayCastHit;

RayCast RayCast_create(Vector3 origin, Vector3 dir, float maxDistance);

void Ray_transform(Transform* transform, RayCast* ray, RayCast* output);
float Ray_calc_distance(RayCast* ray, Vector3* point);

#endif