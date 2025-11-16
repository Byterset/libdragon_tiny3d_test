#ifndef __MATH_PLANE_H__
#define __MATH_PLANE_H__

#include "vector3.h"

typedef struct Plane {
    Vector3 normal;
    float d;
} Plane;

void planeInitWithNormalAndPoint(Plane* plane, const Vector3* normal, const Vector3* point);

bool planeRayIntersection(const Plane* plane, const Vector3* rayOrigin, const Vector3* rayDirection, float* rayDistance);

float planePointDistance(const Plane* plane, const Vector3* point);
void planeProjectPoint(const Plane* plane, const Vector3* point, Vector3* output);

void calculateBarycentricCoords(Vector3* a, Vector3* b, Vector3* c, Vector3* point, Vector3* output);
void evaluateBarycentricCoords(Vector3* a, Vector3* b, Vector3* c, Vector3* bary, Vector3* output);

#endif