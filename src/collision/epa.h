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

/// @brief Solves EPA to find penetration depth and contact information for overlapping objects.
///
/// Starting from a GJK simplex that contains the origin (indicating collision), EPA iteratively
/// expands the polytope toward the Minkowski difference boundary. Each iteration:
///   1. Finds the face closest to the origin (min-heap root)
///   2. Computes a new support point in that face's normal direction
///   3. Tests convergence: if the new point barely extends the polytope, we've found the boundary
///   4. Otherwise, expands the polytope by replacing the closest face with three new faces
///
/// The closest face at convergence defines the penetration normal and depth.
///
/// @param startingSimplex GJK simplex containing the origin (4-point tetrahedron)
/// @param objectA First colliding object
/// @param objectASupport Support function for object A
/// @param objectB Second colliding object
/// @param objectBSum Support function for object B
/// @param result Output: penetration depth, normal, and contact points
/// @return true if penetration info was successfully calculated, false otherwise
bool epaSolve(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, struct EpaResult* result);


/// @brief Swept EPA for continuous collision detection (time of impact calculation).
///
/// Unlike standard EPA which finds penetration for already-overlapping objects, swept EPA
/// finds the moment when a moving object (B) first touches a stationary object (A).
/// The algorithm walks the polytope surface in the sweep direction to find the first contact
/// face, then calculates the exact time and contact geometry.
///
/// @param startingSimplex GJK simplex at the end position (objects overlapping)
/// @param objectA Stationary object
/// @param objectASupport Support function for object A
/// @param objectB Moving object
/// @param objectBSum Support function for object B
/// @param bStart Starting position of object B (modified to first contact position)
/// @param bEnd Ending position of object B
/// @param result Output: contact normal and points at time of impact
/// @return TRUE if valid collision found within the sweep, FALSE otherwise
bool epaSolveSwept(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, Vector3* bStart, Vector3* bEnd, struct EpaResult* result);

#endif