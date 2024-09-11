/**
 * @file aabb.h
 * @brief This file provides the definition of the AABB structure and functions to work with Axis Aligned Bounding Boxes.
 * 
 */

#ifndef _MATH_BOX3D_H
#define _MATH_BOX3D_H

#include "vector3.h"

/**
 * @brief The Axis Aligned Bounding Box (AABB) structure.
 * 
 */
struct AABB {
    struct Vector3 min; /**< The minimum of the Bounding Box or the bottom corner*/
    struct Vector3 max; /**< The maximum of the Bounding Box or the top corner*/
};

int AABBContainsPoint(struct AABB* box, struct Vector3* point);

int AABBHasOverlap(struct AABB* a, struct AABB* b);
void AABBUnion(struct AABB* a, struct AABB* b, struct AABB* out);

void AABBUnionPoint(struct AABB* a, struct Vector3* point, struct AABB* out);

void AABBExtendDirection(struct AABB* a, struct Vector3* direction, struct AABB* out);

void AABBSupportFunction(struct AABB* box, struct Vector3* input, struct Vector3* output);

#endif