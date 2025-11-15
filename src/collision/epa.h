#ifndef __EPA_H__
#define __EPA_H__

#include "gjk.h"

#include <stdbool.h>

/// @brief The resulting structure of the Expanding Polytope Algorithm (EPA)
struct EpaResult {
    Vector3 contactA; // the point on the surface of A that is furthest inside B
    Vector3 contactB; // the point on the surface of B that is furthest inside A
    Vector3 normal; // the contact normal that points from B to A
    float penetration; // by how much do A and B overlap
};

bool epaSolve(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, struct EpaResult* result);
int epaSolveSwept(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, Vector3* bStart, Vector3* bEnd, struct EpaResult* result);
void epaSwapResult(struct EpaResult* result);

#endif