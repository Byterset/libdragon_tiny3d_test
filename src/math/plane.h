#ifndef __MATH_PLANE_H__
#define __MATH_PLANE_H__

#include "vector3.h"

struct Plane {
    Vector3 normal;
    float d;
};

void planeInitWithNormalAndPoint(struct Plane* plane, Vector3* normal, Vector3* point);

int planeRayIntersection(struct Plane* plane, Vector3* rayOrigin, Vector3* rayDirection, float* rayDistance);

float planePointDistance(struct Plane* plane, Vector3* point);
void planeProjectPoint(struct Plane* plane, Vector3* point, Vector3* output);

void calculateBarycentricCoords(Vector3* a, Vector3* b, Vector3* c, Vector3* point, Vector3* output);
void evaluateBarycentricCoords(Vector3* a, Vector3* b, Vector3* c, Vector3* bary, Vector3* output);

#endif