#ifndef __EPA_H__
#define __EPA_H__

#include "gjk.h"

#include <stdbool.h>

struct EpaResult {
    struct Vector3 contactA;
    struct Vector3 contactB;
    // points from A to B
    struct Vector3 normal;
    float penetration;
};

bool epaSolve(struct Simplex* startingSimplex, void* objectA, MinkowskiSum objectASum, void* objectB, MinkowskiSum objectBSum, struct EpaResult* result);
int epaSolveSwept(struct Simplex* startingSimplex, void* objectA, MinkowskiSum objectASum, void* objectB, MinkowskiSum objectBSum, struct Vector3* bStart, struct Vector3* bEnd, struct EpaResult* result);
void epaSwapResult(struct EpaResult* result);

#endif