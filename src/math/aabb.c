/**
 * @file aabb.c
 * @brief Implementation of the AABB structure and functions to work with Axis Aligned Bounding Boxes.
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "aabb.h"
#include <math.h>



/**
 * @brief Checks if a point is contained within an Axis-Aligned Bounding Box (AABB).
 *
 * This function determines whether a given point lies within the boundaries of a specified AABB.
 *
 * @param box A pointer to the AABB structure that defines the bounding box.
 * @param point A pointer to the Vector3 structure that defines the point to be checked.
 * @return Returns 1 if the point is within the AABB, otherwise returns 0.
 */
int AABBContainsPoint(struct AABB* box, struct Vector3* point) {
    return box->min.x < point->x &&
        box->min.y < point->y &&
        box->min.z < point->z &&
        box->max.x > point->x &&
        box->max.y > point->y &&
        box->max.z > point->z;
        
}

/**
 * @brief Checks if an Axis-Aligned Bounding Box (AABB) contains another AABB.
 * 
 * @param a 
 * @param b 
 * @return int 
 */
int AABBContainsAABB(struct AABB *a, struct AABB *b)
{
    return (a->min.x <= b->min.x && a->min.y <= b->min.y && a->min.z <= b->min.z &&
            a->max.x >= b->max.x && a->max.y >= b->max.y && a->max.z >= b->max.z);
}

/**
 * @brief Checks if two Axis-Aligned Bounding Boxes (AABB) overlap.
 *
 * This function determines whether two AABBs intersect by comparing their
 * minimum and maximum coordinates along the x, y, and z axes.
 *
 * @param a Pointer to the first AABB structure.
 * @param b Pointer to the second AABB structure.
 * @return int Returns 1 if the AABBs overlap, 0 otherwise.
 */
int AABBHasOverlap(struct AABB* a, struct AABB* b) {
    return a->min.x <= b->max.x && a->max.x >= b->min.x &&
        a->min.y <= b->max.y && a->max.y >= b->min.y &&
        a->min.z <= b->max.z && a->max.z >= b->min.z;
}

/// @brief Checks if an Axis-Aligned Bounding Box (AABB) intersects with a ray.
/// @param box Pointer to the AABB structure.
/// @param origin Origin of the ray.
/// @param direction Direction of the ray.
/// @param max_distance Maximum distance of the ray after which intersections should not be considered.
/// @return 
int AABBIntersectsRay(struct AABB* box, struct RayCast* ray){
    float tmin = -__FLT_MAX__;
    float tmax = __FLT_MAX__;
    float t1, t2;
    float epsilon = __FLT_EPSILON__;
    if (fabsf(ray->dir.x) < epsilon)
    {
        if (ray->origin.x < box->min.x || ray->origin.x > box->max.x)
            return 0;
    }
    else
    {
        t1 = (box->min.x - ray->origin.x) / ray->dir.x;
        t2 = (box->max.x - ray->origin.x) / ray->dir.x;
        tmin = fmaxf(tmin, fminf(t1, t2));
        tmax = fminf(tmax, fmaxf(t1, t2));
    }
    if (tmin > tmax)
        return 0;
    if (fabsf(ray->dir.y) < epsilon)
    {
        if (ray->origin.y < box->min.y || ray->origin.y > box->max.y)
            return 0;
    }
    else
    {
        t1 = (box->min.y - ray->origin.y) / ray->dir.y;
        t2 = (box->max.y - ray->origin.y) / ray->dir.y;
        tmin = fmaxf(tmin, fminf(t1, t2));
        tmax = fminf(tmax, fmaxf(t1, t2));
    }
    if (tmin > tmax)
        return 0;
    if (fabsf(ray->dir.z) < epsilon)
    {
        if (ray->origin.z < box->min.z || ray->origin.z > box->max.z)
            return 0;
    }
    else
    {
        t1 = (box->min.z - ray->origin.z) / ray->dir.z;
        t2 = (box->max.z - ray->origin.z) / ray->dir.z;
        tmin = fmaxf(tmin, fminf(t1, t2));
        tmax = fminf(tmax, fmaxf(t1, t2));
    }
    if (tmin > tmax)
        return 0;
    return (tmin >= 0 && tmin < ray->maxDistance);
}

/**
 * @brief Computes the union of two axis-aligned bounding boxes (AABBs).
 *
 * This function takes two AABBs, `a` and `b`, and computes their union,
 * storing the result in the output AABB `out`. The union of two AABBs
 * is the smallest AABB that contains both `a` and `b`.
 *
 * @param a Pointer to the first AABB.
 * @param b Pointer to the second AABB.
 */
struct AABB AABBUnion(struct AABB* a, struct AABB* b) {
    struct AABB out;
    vector3Max(&a->max, &b->max, &out.max);
    vector3Min(&a->min, &b->min, &out.min);
    return out;
}

float AABBGetArea(struct AABB aabb){
    float x = aabb.max.x - aabb.min.x;
    float y = aabb.max.y - aabb.min.y;
    float z = aabb.max.z - aabb.min.z;
    // Surface area of a rectangular prism: 2(w*h + h*d + d*w)
    return 2 * (x * y + x * z + y * z);
}

/**
 * @brief Computes the union of an axis-aligned bounding box (AABB) and a point.
 *
 * This function takes an AABB `a` and a point `point`, and computes the union
 * of the AABB and the point. The result is stored in the output AABB `out`.
 *
 * @param a Pointer to the AABB.
 * @param point Pointer to the point.
 */
struct AABB AABBUnionPoint(struct AABB* a, struct Vector3* point) {
    struct AABB out;
    vector3Max(&a->max, point, &out.max);
    vector3Min(&a->min, point, &out.min);
    return out;
}

/**
 * @brief Extends an axis-aligned bounding box (AABB) in a specified direction.
 *
 * This function takes an AABB `a` and a direction vector `direction`, and
 * extends the AABB in the specified direction. The result is stored in the
 * output AABB `out`. The extension is equal to the magnitude of the direction.
 *
 * @param a Pointer to the AABB to be extended.
 * @param direction Pointer to the direction vector.
 * @param out Pointer to the output AABB where the result will be stored.
 */
void AABBExtendDirection(struct AABB* a, struct Vector3* direction, struct AABB* out) {
    *out = *a;

    if (direction->x > 0.0) {
        out->max.x += direction->x;
    } else {
        out->min.x += direction->x;
    }

    if (direction->y > 0.0) {
        out->max.y += direction->y;
    } else {
        out->min.y += direction->y;
    }

    if (direction->z > 0.0) {
        out->max.z += direction->z;
    } else {
        out->min.z += direction->z;
    }
}

/**
 * @brief Computes the support function for an axis-aligned bounding box (AABB).
 *
 * This function computes the support function for an AABB in the specified direction.
 * The support function returns the point on the AABB that is farthest in the given direction.
 *
 * @param box Pointer to the AABB.
 * @param input Pointer to the direction vector.
 * @param output Pointer to the output vector where the result will be stored.
 */
void AABBSupportFunction(struct AABB* box, struct Vector3* input, struct Vector3* output) {
    output->x = input->x > 0.0f ? box->max.x : box->min.x;
    output->y = input->y > 0.0f ? box->max.y : box->min.y;
    output->z = input->z > 0.0f ? box->max.z : box->min.z;
} 