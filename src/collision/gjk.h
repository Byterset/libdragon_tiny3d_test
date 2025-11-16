#ifndef __GJK_H___
#define __GJK_H___

#include "../math/mathf.h"
#include "../math/vector3.h"
#include "../math/quaternion.h"

typedef void (*gjk_support_function)(const void* data, const Vector3* direction, Vector3* output);

#define GJK_MAX_SIMPLEX_SIZE    4

struct Simplex {
    Vector3 points[GJK_MAX_SIMPLEX_SIZE];
    Vector3 objectAPoint[GJK_MAX_SIMPLEX_SIZE];
    short nPoints;
};

void simplexInit(struct Simplex* simplex);
int simplexCheck(struct Simplex* simplex, Vector3* nextDirection);

int gjkCheckForOverlap(struct Simplex* simplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSupport, Vector3* firstDirection);

#endif