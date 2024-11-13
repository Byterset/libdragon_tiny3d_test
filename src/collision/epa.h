#ifndef __EPA_H__
#define __EPA_H__

#include "gjk.h"

#include <stdbool.h>

struct EpaResult {
    Vector3 contactA;
    Vector3 contactB;
    // points from A to B
    Vector3 normal;
    float penetration;
};

bool epaSolve(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, struct EpaResult* result);
int epaSolveSwept(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, Vector3* bStart, Vector3* bEnd, struct EpaResult* result);
void epaSwapResult(struct EpaResult* result);

#endif