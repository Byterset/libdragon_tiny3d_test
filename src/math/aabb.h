/**
 * @file aabb.h
 * @brief This file provides the definition of the AABB structure and functions to work with Axis Aligned Bounding Boxes.
 * 
 */

#ifndef _MATH_BOX3D_H
#define _MATH_BOX3D_H

#include "vector3.h"
#include "../collision/raycast.h"

/**
 * @brief The Axis Aligned Bounding Box (AABB) structure.
 * 
 */
typedef struct AABB {
    Vector3 min; /**< The minimum of the Bounding Box or the bottom corner*/
    Vector3 max; /**< The maximum of the Bounding Box or the top corner*/
} AABB;

int AABBContainsPoint(AABB* box, Vector3* point);

int AABBContainsAABB(AABB* a, AABB* b);

int AABBHasOverlap(AABB* a, AABB* b);

int AABBIntersectsRay(AABB* box, raycast* ray);

float AABBGetArea(AABB aabb);

AABB AABBUnion(AABB* a, AABB* b);

AABB AABBUnionPoint(AABB* a, Vector3* point);

AABB AABBFromTriangle(Vector3* a, Vector3* b, Vector3* c);

void AABBExtendDirection(AABB* a, Vector3* direction, AABB* out);

void AABBSupportFunction(AABB* box, Vector3* input, Vector3* output);

#endif