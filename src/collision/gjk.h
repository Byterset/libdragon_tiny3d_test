#ifndef __GJK_H___
#define __GJK_H___

#include "../math/mathf.h"
#include "../math/vector3.h"
#include "../math/quaternion.h"

/// @brief A GJK support function that returns the furthest point in a given direction on the boundary of a convex shape
typedef void (*gjk_support_function)(const void* data, const Vector3* direction, Vector3* output);

#define GJK_MAX_SIMPLEX_SIZE    4

/// @brief simplex struct (Tetrahedron) for use in the solving of the GJK 
struct Simplex {
    Vector3 points[GJK_MAX_SIMPLEX_SIZE];
    Vector3 objectAPoint[GJK_MAX_SIMPLEX_SIZE];
    short nPoints;
};


/// @brief Initialize a Simplex
/// @param simplex 
inline void simplexInit(struct Simplex* simplex) {
    simplex->nPoints = 0;
}


/// @brief Checks the current simplex to determine if it encloses the origin and updates the search direction.
///
/// @param simplex Pointer to the current simplex structure.
/// @param nextDirection Pointer to the vector that will be updated with the next search direction.
/// @return TRUE if the simplex encloses the origin, FALSE otherwise.
///
/// The function performs the following steps:
/// 1. Retrieves the last added point in the simplex and calculates the vector from this point to the origin.
/// 2. Depending on the number of points in the simplex, different checks and updates are performed:
///    - If the simplex has 2 points:
///      - Calculates the vector from the last added point to the other point.
///      - Uses the triple product to find the perpendicular direction to the line segment.
///      - If the magnitude of the new direction is very small, it uses a perpendicular vector.
///    - If the simplex has 3 points:
///      - Calculates the normal of the triangle formed by the points.
///      - Checks if the origin is in front of or behind the triangle.
///      - Depending on the position of the origin, updates the simplex and the next search direction.
///      - If the origin is behind the triangle, changes the winding of the triangle.
///    - If the simplex has 4 points:
///      - Iterates through the faces of the tetrahedron formed by the points.
///      - Determines how many faces have the origin in front of them.
///      - If all faces are behind the origin, the origin is enclosed.
///      - If one face is in front, updates the simplex and the next search direction.
///      - If two faces are in front, updates the simplex and the next search direction.
///      - If three faces are in front, resets the simplex to a single point and updates the direction.
bool simplexCheck(struct Simplex* simplex, Vector3* nextDirection);


/// @brief Takes two objects and their support functions and checks if they overlap using the GJK algorithm.
/// @param simplex pointer to the simplex structure to use for the GJK algorithm and EPA
/// @param objectA first physics object
/// @param objectASupport support function for the first object
/// @param objectB second physics object
/// @param objectBSupport support function for the second object
/// @param firstDirection initial direction to search for the origin
/// @return TRUE if the objects overlap, FALSE otherwise
bool gjkCheckForOverlap(struct Simplex* simplex, const void* objectA, gjk_support_function objectASupport, const void* objectB, gjk_support_function objectBSupport, Vector3* firstDirection);

#endif