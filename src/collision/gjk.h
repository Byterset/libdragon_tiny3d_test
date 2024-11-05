#ifndef __GJK_H___
#define __GJK_H___

#include "../math/mathf.h"
#include "../math/vector3.h"
#include "../math/quaternion.h"

typedef void (*gjk_support_function)(void* data, struct Vector3* direction, struct Vector3* output);

#define MAX_SIMPLEX_SIZE    4

struct Simplex {
    struct Vector3 points[MAX_SIMPLEX_SIZE];
    struct Vector3 objectAPoint[MAX_SIMPLEX_SIZE];
    short nPoints;
};

void simplexInit(struct Simplex* simplex);
int simplexCheck(struct Simplex* simplex, struct Vector3* nextDirection);

int gjkCheckForOverlap(struct Simplex* simplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSupport, struct Vector3* firstDirection);

#endif