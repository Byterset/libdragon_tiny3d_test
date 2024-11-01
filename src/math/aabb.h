/**
 * @file aabb.h
 * @brief This file provides the definition of the AABB structure and functions to work with Axis Aligned Bounding Boxes.
 * 
 */

#ifndef _MATH_BOX3D_H
#define _MATH_BOX3D_H

#include "vector3.h"
#include "ray.h"

/**
 * @brief The Axis Aligned Bounding Box (AABB) structure.
 * 
 */
struct AABB {
    struct Vector3 min; /**< The minimum of the Bounding Box or the bottom corner*/
    struct Vector3 max; /**< The maximum of the Bounding Box or the top corner*/
};

int AABBContainsPoint(struct AABB* box, struct Vector3* point);

int AABBContainsAABB(struct AABB* a, struct AABB* b);

int AABBHasOverlap(struct AABB* a, struct AABB* b);

int AABBIntersectsRay(struct AABB* box, struct RayCast* ray);

float AABBGetArea(struct AABB aabb);

struct AABB AABBUnion(struct AABB* a, struct AABB* b);

struct AABB AABBUnionPoint(struct AABB* a, struct Vector3* point);

void AABBExtendDirection(struct AABB* a, struct Vector3* direction, struct AABB* out);

void AABBSupportFunction(struct AABB* box, struct Vector3* input, struct Vector3* output);

#endif