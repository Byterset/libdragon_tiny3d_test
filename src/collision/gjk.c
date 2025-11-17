/**
 * Gilbert–Johnson–Keerthi distance algorithm (GJK) implementation for collision detection.
 *
 * Efficiently determines if two convex shapes overlap or not by attempting to build a Simplex (Tetrahedron for 3D GJK)
 * that contains the origin.
 */

#include "gjk.h"

#define GJK_MAX_ITERATIONS  18

Vector3* simplexAddPoint(struct Simplex* simplex, Vector3* aPoint, Vector3* bPoint) {
    if (simplex->nPoints == GJK_MAX_SIMPLEX_SIZE) {
        // SHOULD never happen, but just in case
        return NULL;
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


bool simplexCheck(struct Simplex* simplex, Vector3* nextDirection) {
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

        return false;
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

            return false;
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

            return false;
        }

        if (vector3Dot(&normal, &aToOrigin) > 0.0f) {
            *nextDirection = normal;
            return false;
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
            return true;
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

    return false;
}


bool gjkCheckForOverlap(struct Simplex* simplex, const void* objectA, gjk_support_function objectASupport, const void* objectB, gjk_support_function objectBSupport, Vector3* firstDirection) {
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
            return false;
        }
        
        if (vector3Dot(addedPoint, &nextDirection) <= 0.0f) {
            return false;
        }

        if (simplexCheck(simplex, &nextDirection)) {
            return true;
        }

    }
    // if we reach here, we have not found a solution in the maximum number of iterations
    return false;
}