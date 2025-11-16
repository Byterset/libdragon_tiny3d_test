#include "gjk.h"

#define GJK_MAX_ITERATIONS  18

void simplexInit(struct Simplex* simplex) {
    simplex->nPoints = 0;
}

Vector3* simplexAddPoint(struct Simplex* simplex, Vector3* aPoint, Vector3* bPoint) {
    if (simplex->nPoints == GJK_MAX_SIMPLEX_SIZE) {
        // SHOULD never happen, but just in case
        return 0;
    }

    int index = simplex->nPoints;

    simplex->objectAPoint[index] = *aPoint;
    vector3Sub(&simplex->objectAPoint[index], bPoint, &simplex->points[index]);
    ++simplex->nPoints;

    return &simplex->points[index];
}

void simplexMovePoint(struct Simplex* simplex, int to, int from) {
    simplex->points[to] = simplex->points[from];
    simplex->objectAPoint[to] = simplex->objectAPoint[from];
}

/**
 * Checks the current simplex to determine if it encloses the origin and updates the search direction.
 *
 * @param simplex Pointer to the current simplex structure.
 * @param nextDirection Pointer to the vector that will be updated with the next search direction.
 * @return 1 if the simplex encloses the origin, 0 otherwise.
 *
 * The function performs the following steps:
 * 1. Retrieves the last added point in the simplex and calculates the vector from this point to the origin.
 * 2. Depending on the number of points in the simplex, different checks and updates are performed:
 *    - If the simplex has 2 points:
 *      - Calculates the vector from the last added point to the other point.
 *      - Uses the triple product to find the perpendicular direction to the line segment.
 *      - If the magnitude of the new direction is very small, it uses a perpendicular vector.
 *    - If the simplex has 3 points:
 *      - Calculates the normal of the triangle formed by the points.
 *      - Checks if the origin is in front of or behind the triangle.
 *      - Depending on the position of the origin, updates the simplex and the next search direction.
 *      - If the origin is behind the triangle, changes the winding of the triangle.
 *    - If the simplex has 4 points:
 *      - Iterates through the faces of the tetrahedron formed by the points.
 *      - Determines how many faces have the origin in front of them.
 *      - If all faces are behind the origin, the origin is enclosed.
 *      - If one face is in front, updates the simplex and the next search direction.
 *      - If two faces are in front, updates the simplex and the next search direction.
 *      - If three faces are in front, resets the simplex to a single point and updates the direction.
 */
int simplexCheck(struct Simplex* simplex, Vector3* nextDirection) {
    Vector3* lastAdded = &simplex->points[simplex->nPoints - 1];
    Vector3 aToOrigin;
    vector3Negate(lastAdded, &aToOrigin);

    if (simplex->nPoints == 2) { 
        Vector3 lastAddedToOther;
        vector3Sub(&simplex->points[0], lastAdded, &lastAddedToOther);
        vector3TripleProduct(&lastAddedToOther, &aToOrigin, &lastAddedToOther, nextDirection);

        if (vector3MagSqrd(nextDirection) <= 0.0000001f) {
            vector3Perpendicular(&lastAddedToOther, nextDirection);
        }

        return 0;
    } else if (simplex->nPoints == 3) {
        Vector3 normal;
        Vector3 ab;
        vector3Sub(&simplex->points[1], lastAdded, &ab);
        Vector3 ac;
        vector3Sub(&simplex->points[0], lastAdded, &ac);

        vector3Cross(&ab, &ac, &normal);

        Vector3 dirCheck;
        vector3Cross(&ab, &normal, &dirCheck);

        if (vector3Dot(&dirCheck, &aToOrigin) > 0.0f) {
            vector3TripleProduct(&ab, &aToOrigin, &ab, nextDirection);

            if (vector3MagSqrd(nextDirection) <= 0.0000001f) {
                *nextDirection = normal;
            }

            // remove c
            simplexMovePoint(simplex, 0, 1);
            simplexMovePoint(simplex, 1, 2);
            
            simplex->nPoints = 2;

            return 0;
        }

        vector3Cross(&normal, &ac, &dirCheck);

        if (vector3Dot(&dirCheck, &aToOrigin) > 0.0f) {
            vector3TripleProduct(&ac, &aToOrigin, &ac, nextDirection);

            if (vector3MagSqrd(nextDirection) <= 0.0000001f) {
                *nextDirection = normal;
            }

            // remove b
            simplexMovePoint(simplex, 1, 2);
            simplex->nPoints = 2;

            return 0;
        }

        if (vector3Dot(&normal, &aToOrigin) > 0.0f) {
            *nextDirection = normal;
            return 0;
        }

        // change triangle winding
        // temporarily use unused vertex 4
        simplexMovePoint(simplex, 3, 0);
        simplexMovePoint(simplex, 0, 1);
        simplexMovePoint(simplex, 1, 3);
        vector3Negate(&normal, nextDirection);
    } else if (simplex->nPoints == 4) {
        int lastBehindIndex = -1;
        int lastInFrontIndex = -1;
        int isFrontCount = 0;

        Vector3 normals[3];

        for (int i = 0; i < 3; ++i) {
            Vector3 firstEdge;
            Vector3 secondEdge;
            vector3Sub(lastAdded, &simplex->points[i], &firstEdge);
            vector3Sub(i == 2 ? &simplex->points[0] : &simplex->points[i + 1], &simplex->points[i], &secondEdge);
            vector3Cross(&firstEdge, &secondEdge, &normals[i]);

            if (vector3Dot(&aToOrigin, &normals[i]) > 0.0f) {
                ++isFrontCount;
                lastInFrontIndex = i;
            } else {
                lastBehindIndex = i;
            }
        }

        if (isFrontCount == 0) { // origin is enclosed thus the objects overlap
            return 1;
        } else if (isFrontCount == 1) { // origin is in front of one face so we need to update the simplex
            *nextDirection = normals[lastInFrontIndex];

            if (lastInFrontIndex == 1) {
                simplexMovePoint(simplex, 0, 1);
                simplexMovePoint(simplex, 1, 2);
            } else if (lastInFrontIndex == 2) {
                simplexMovePoint(simplex, 1, 0);
                simplexMovePoint(simplex, 0, 2);
            }

            simplexMovePoint(simplex, 2, 3);
            simplex->nPoints = 3;
        } else if (isFrontCount == 2) { // origin is in front of two faces so we need to update the simplex
            if (lastBehindIndex == 0) {
                simplexMovePoint(simplex, 0, 2);
            } else if (lastBehindIndex == 2) {
                simplexMovePoint(simplex, 0, 1);
            }

            simplexMovePoint(simplex, 1, 3);
            simplex->nPoints = 2;

            Vector3 ab;
            vector3Sub(&simplex->points[0], &simplex->points[1], &ab);

            vector3TripleProduct(&ab, &aToOrigin, &ab, nextDirection);

            if (vector3MagSqrd(nextDirection) <= 0.0000001f) {
                vector3Perpendicular(&ab, nextDirection);
            }
        } else { // origin is in front of three faces, should never happen
            // if it does reset the simplex to a single point and update the direction
            simplexMovePoint(simplex, 0, 3);
            simplex->nPoints = 1;
            *nextDirection = aToOrigin;
        }
    }

    return 0;
}

/// @brief Takes two objects and their support functions and checks if they overlap using the GJK algorithm.
/// @param simplex pointer to the simplex structure to use for the GJK algorithm and EPA
/// @param objectA first physics object
/// @param objectASupport support function for the first object
/// @param objectB second physics object
/// @param objectBSupport support function for the second object
/// @param firstDirection initial direction to search for the origin
/// @return 1 if the objects overlap, 0 otherwise
int gjkCheckForOverlap(struct Simplex* simplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSupport, Vector3* firstDirection) {
    Vector3 aPoint;
    Vector3 bPoint;
    Vector3 nextDirection;

    simplexInit(simplex);

    // if for whatever reason the first direction is zero, we need to pick a new one
    if (vector3IsZero(firstDirection)) {
        *firstDirection = gRight;
    }

    objectASupport(objectA, firstDirection, &aPoint);
    vector3Negate(firstDirection, &nextDirection);

    objectBSupport(objectB, &nextDirection, &bPoint);
    simplexAddPoint(simplex, &aPoint, &bPoint);

    for (int iteration = 0; iteration < GJK_MAX_ITERATIONS; ++iteration) {
        Vector3 reverseDirection;
        vector3Negate(&nextDirection, &reverseDirection);
        objectASupport(objectA, &nextDirection, &aPoint);
        objectBSupport(objectB, &reverseDirection, &bPoint);

        Vector3* addedPoint = simplexAddPoint(simplex, &aPoint, &bPoint);

        if (!addedPoint) {
            return 0;
        }
        
        if (vector3Dot(addedPoint, &nextDirection) <= 0.0f) {
            return 0;
        }

        if (simplexCheck(simplex, &nextDirection)) {
            return 1;
        }

    }
    // if we reach here, we have not found a solution in the maximum number of iterations
    return 0;
}