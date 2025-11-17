/**
 * Expanding Polytope Algorithm (EPA) implementation for collision detection.
 *
 * EPA refines the penetration information after GJK determines that two convex shapes intersect.
 * While GJK efficiently detects collision, it only provides a simplex containing the origin.
 * EPA expands this simplex into a polytope to find:
 *   - The exact penetration depth
 *   - The collision normal
 *   - Contact points on both objects
 *
 * The algorithm works by iteratively expanding the polytope toward the origin, finding the
 * face closest to the origin. This closest face defines the minimum translation vector needed
 * to separate the objects.
 */

#include "epa.h"

#include <assert.h>
#include "../math/plane.h"
#include "../math/mathf.h"

// Limit iterations to prevent infinite loops while allowing enough refinement for accurate results
#define EPA_MAX_ITERATIONS  10

#define EPA_MAX_SIMPLEX_POINTS      (4 + EPA_MAX_ITERATIONS)
#define EPA_MAX_SIMPLEX_TRIANGLES   (4 + EPA_MAX_ITERATIONS * 2)

// Circular indexing for triangle edges (0->1, 1->2, 2->0)
static const unsigned char NEXT_FACE_LUT[3] = {1, 2, 0};
#define NEXT_FACE(index) (NEXT_FACE_LUT[index])

/**
 * Triangle topology data for the expanding polytope.
 * Stored as a union to ensure proper alignment while maintaining compact storage.
 */
union SimplexTriangleIndexData {
    struct {
        unsigned char indices[3];           // Vertex indices forming this triangle
        unsigned char adjacentFaces[3];     // Adjacent triangle indices for each edge
        unsigned char oppositePoints[3];    // For each edge, the local index (0-2) of the opposite vertex in the adjacent triangle
    };
    int alignment;  // Ensures the union is aligned for efficient memory access
};

/**
 * Represents a face of the expanding polytope.
 * The polytope is a convex hull approximation in Minkowski difference space.
 */
struct SimplexTriangle {
    union SimplexTriangleIndexData indexData;
    float distanceToOrigin;  // Distance of this triangle to origin. Critical for finding the closest face, which determines penetration depth
    Vector3 normal;
};

enum SimplexFlags {
    SimplexFlagsSkipDistance = (1 << 0),  // Used in swept EPA to defer distance calculations until needed
};

/**
 * The expanding polytope structure.
 * Maintains both the Minkowski difference points and original object A points for contact calculation.
 * Uses a min-heap to efficiently track the closest face to the origin.
 */
struct ExpandingSimplex {
    Vector3 points[EPA_MAX_SIMPLEX_POINTS];      // Points in Minkowski difference space (A - B)
    Vector3 aPoints[EPA_MAX_SIMPLEX_POINTS];     // Corresponding points on object A (needed for final contact point)
    struct SimplexTriangle triangles[EPA_MAX_SIMPLEX_TRIANGLES];
    short pointCount;
    short triangleCount;
    unsigned char triangleHeap[EPA_MAX_SIMPLEX_TRIANGLES];           // Min-heap by distance to origin
    unsigned char triangleToHeapIndex[EPA_MAX_SIMPLEX_TRIANGLES];    // Inverse lookup for heap updates
    short flags;
};


#define GET_PARENT_INDEX(heapIndex) (((heapIndex) - 1) >> 1)
#define GET_CHILD_INDEX(heapIndex, childHeapIndex)  (((heapIndex) << 1) + 1 + (childHeapIndex))
#define EXPANDING_SIMPLEX_GET_DISTANCE(simplex, triangleIndex)  ((simplex)->triangles[triangleIndex].distanceToOrigin)

/**
 * @brief Validates the min-heap property for debugging purposes.
 *
 * Ensures all children have distances >= their parent's distance, which is critical
 * for efficiently finding the closest face to the origin.
 *
 * @param simplex The expanding simplex to validate
 * @return 1 if heap is valid, 0 otherwise
 */
int validateHeap(struct ExpandingSimplex* simplex) {
    for (int i = 1; i < simplex->triangleCount; ++i) {
        int parentIndex = GET_PARENT_INDEX(i);

        if (simplex->triangles[simplex->triangleHeap[i]].distanceToOrigin < simplex->triangles[simplex->triangleHeap[parentIndex]].distanceToOrigin) {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief Validates polytope topology and convexity for debugging purposes.
 *
 * Verifies that adjacency relationships are bidirectional and that the polytope remains convex.
 * Convexity is essential because EPA assumes the polytope faces point outward from the origin.
 *
 * @param simplex The expanding simplex to validate
 * @return 1 if topology is valid and polytope is convex, 0 otherwise
 */
int validateExpandingSimplex(struct ExpandingSimplex* simplex) {
    for (int triangleIndex = 0; triangleIndex < simplex->triangleCount; ++triangleIndex) {
        struct SimplexTriangle* triangle = &simplex->triangles[triangleIndex];

        for (int index = 0; index < 3; ++index) {
            struct SimplexTriangle* adjacent = &simplex->triangles[triangle->indexData.adjacentFaces[index]];

            unsigned char nextFromOpposite = NEXT_FACE(triangle->indexData.oppositePoints[index]);

            // Verify bidirectional adjacency (if A says B is adjacent, B must say A is adjacent)
            if (adjacent->indexData.adjacentFaces[nextFromOpposite] != triangleIndex) {
                return 0;
            }

            Vector3 offset;
            vector3Sub(
                &simplex->points[adjacent->indexData.indices[triangle->indexData.oppositePoints[index]]],
                &simplex->points[triangle->indexData.indices[index]],
                &offset
            );

            // Verify convexity: opposite vertex must be behind or on the face plane
            if (vector3Dot(&offset, &triangle->normal) > 0.000001f) {
                return 0;
            }
        }
    }

    return 1;
}

/**
 * @brief Adds a new support point to the expanding simplex.
 *
 * @param simplex The expanding simplex
 * @param aPoint Point on the surface of object A
 * @param pointDiff Minkowski difference point (A - B)
 */
static inline void expandingSimplexAddPoint(struct ExpandingSimplex* simplex, Vector3* aPoint, Vector3* pointDiff) {
    short result = simplex->pointCount;
    simplex->aPoints[result] = *aPoint;
    simplex->points[result] = *pointDiff;
    ++simplex->pointCount;
}

/**
 * @brief Restores min-heap property by moving an element upward (toward root).
 *
 * Used when a triangle's distance decreases. Moves the element up the heap until
 * it finds its correct position where parent distance <= current distance.
 * The inverse mapping is maintained to allow O(log n) updates to any triangle.
 *
 * @param simplex The expanding simplex
 * @param heapIndex Starting position in the heap
 * @return Final position of the element after sifting
 */
int expandingSimplexSiftDownHeap(struct ExpandingSimplex* simplex, int heapIndex) {
    int parentHeapIndex = GET_PARENT_INDEX(heapIndex);
    float currentDistance = EXPANDING_SIMPLEX_GET_DISTANCE(simplex, simplex->triangleHeap[heapIndex]);

    while (heapIndex > 0) {
        if (currentDistance >= EXPANDING_SIMPLEX_GET_DISTANCE(simplex, simplex->triangleHeap[parentHeapIndex])) {
            break;
        }

        unsigned char tmp = simplex->triangleHeap[heapIndex];
        simplex->triangleHeap[heapIndex] = simplex->triangleHeap[parentHeapIndex];
        simplex->triangleHeap[parentHeapIndex] = tmp;

        simplex->triangleToHeapIndex[simplex->triangleHeap[heapIndex]] = heapIndex;
        simplex->triangleToHeapIndex[simplex->triangleHeap[parentHeapIndex]] = parentHeapIndex;

        heapIndex = parentHeapIndex;
        parentHeapIndex = GET_PARENT_INDEX(heapIndex);
    }

    return heapIndex;
}

/**
 * @brief Restores min-heap property by moving an element downward (toward leaves).
 *
 * Used when a triangle's distance increases. Moves the element down by swapping with
 * the smaller child until the heap property is restored.
 *
 * @param simplex The expanding simplex
 * @param heapIndex Starting position in the heap
 * @return Final position of the element after sifting
 */
int expandingSimplexSiftUpHeap(struct ExpandingSimplex* simplex, int heapIndex) {
    float currentDistance = EXPANDING_SIMPLEX_GET_DISTANCE(simplex, simplex->triangleHeap[heapIndex]);

    while (heapIndex < simplex->triangleCount) {
        int swapWithChild = -1;

        int childHeapIndex = GET_CHILD_INDEX(heapIndex, 0);

        if (childHeapIndex >= simplex->triangleCount) {
            break;
        }

        float childDistance = EXPANDING_SIMPLEX_GET_DISTANCE(simplex, simplex->triangleHeap[childHeapIndex]);

        if (childDistance < currentDistance) {
            swapWithChild = childHeapIndex;
        }

        float otherChildDistance = EXPANDING_SIMPLEX_GET_DISTANCE(simplex, simplex->triangleHeap[childHeapIndex + 1]);

        // Choose the smaller child to maintain min-heap property
        if (childHeapIndex + 1 < simplex->triangleCount && otherChildDistance < currentDistance && otherChildDistance < childDistance) {
            swapWithChild = childHeapIndex + 1;
        }

        if (swapWithChild == -1) {
            break;
        }

        unsigned char tmp = simplex->triangleHeap[heapIndex];
        simplex->triangleHeap[heapIndex] = simplex->triangleHeap[swapWithChild];
        simplex->triangleHeap[swapWithChild] = tmp;

        simplex->triangleToHeapIndex[simplex->triangleHeap[heapIndex]] = heapIndex;
        simplex->triangleToHeapIndex[simplex->triangleHeap[swapWithChild]] = swapWithChild;

        heapIndex = swapWithChild;
    }

    return heapIndex;
}

/**
 * @brief Restores heap property when a triangle's distance changes.
 *
 * Tries sifting up first (more common case), then sifts down if needed.
 * This is necessary after edge rotations or distance recalculations.
 *
 * @param simplex The expanding simplex
 * @param heapIndex Position in heap to fix
 */
void expandingSimplexFixHeap(struct ExpandingSimplex* simplex, int heapIndex) {
    int nextHeapIndex = expandingSimplexSiftUpHeap(simplex, heapIndex);

    if (nextHeapIndex != heapIndex) {
        return;
    }

    expandingSimplexSiftDownHeap(simplex, nextHeapIndex);
}

/**
 * @brief Initializes the unnormalized normal vector for a triangle face.
 *
 * Computes the cross product of two edges. The normal is left unnormalized
 * here for efficiency; it's normalized later only when needed.
 *
 * @param simplex The expanding simplex
 * @param triangle Triangle to initialize normal for
 */
static inline void expandingSimplexTriangleInitNormal(struct ExpandingSimplex* simplex, struct SimplexTriangle* triangle) {
    Vector3 edgeB;
    vector3Sub(&simplex->points[triangle->indexData.indices[1]], &simplex->points[triangle->indexData.indices[0]], &edgeB);
    Vector3 edgeC;
    vector3Sub(&simplex->points[triangle->indexData.indices[2]], &simplex->points[triangle->indexData.indices[0]], &edgeC);

    vector3Cross(&edgeB, &edgeC, &triangle->normal);
}

/**
 * @brief Checks if the origin projects onto a specific edge of the triangle.
 *
 * If the origin's closest point is on this edge rather than the face interior,
 * the distance is calculated to that edge. This handles degenerate cases where
 * the origin is outside the triangle's Voronoi region.
 *
 * @param simplex The expanding simplex
 * @param triangle Triangle to check
 * @param index Edge index (0-2) to check
 * @return 1 if origin projects onto this edge, 0 otherwise
 */
int expandingSimplexTriangleCheckEdge(struct ExpandingSimplex* simplex, struct SimplexTriangle* triangle, int index) {
    Vector3* pointA = &simplex->points[triangle->indexData.indices[index]];

    Vector3 edge;
    vector3Sub(&simplex->points[triangle->indexData.indices[NEXT_FACE(index)]], pointA, &edge);
    Vector3 toOrigin;
    vector3Negate(pointA, &toOrigin);

    Vector3 crossCheck;
    vector3Cross(&edge, &toOrigin, &crossCheck);

    // Check if origin lies in the Voronoi region of this edge using the triple product test
    if (vector3Dot(&crossCheck, &triangle->normal) >= 0.0f) {
        return 0;
    }

    // Clamp projection to edge bounds and find closest point on edge
    float edgeLerp = vector3Dot(&toOrigin, &edge);
    float edgeMagSqrd = vector3MagSqrd(&edge);

    if (edgeLerp < 0.0f) {
        edgeLerp = 0.0f;
    } else if (edgeLerp > edgeMagSqrd) {
        edgeLerp = 1.0f;
    } else {
        edgeLerp /= edgeMagSqrd;
    }

    Vector3 nearestPoint;
    vector3AddScaled(pointA, &edge, edgeLerp, &nearestPoint);

    triangle->distanceToOrigin = vector3Mag(&nearestPoint);

    return 1;
}

/**
 * @brief Computes the distance from the origin to the triangle.
 *
 * First checks if the origin projects onto any edge (outside the face's Voronoi region).
 * If not, the distance is the perpendicular distance to the face plane.
 * This distance determines which face is closest and drives the EPA iteration.
 *
 * @param simplex The expanding simplex
 * @param triangle Triangle to calculate distance for
 */
void expandingSimplexTriangleDetermineDistance(struct ExpandingSimplex* simplex, struct SimplexTriangle* triangle) {
    vector3Normalize(&triangle->normal, &triangle->normal);

    for (int i = 0; i < 3; ++i) {
        if (expandingSimplexTriangleCheckEdge(simplex, triangle, i)) {
            return;
        }
    }
    
    triangle->distanceToOrigin = vector3Dot(&triangle->normal, &simplex->points[triangle->indexData.indices[0]]);
}

/**
 * @brief Rotates a shared edge between two triangles to maintain polytope convexity.
 *
 * Edge rotation (edge flip) is a fundamental operation in maintaining a convex polytope.
 * When a vertex from an adjacent triangle lies outside the current triangle's plane,
 * the shared edge must be rotated to restore convexity. This is analogous to Delaunay
 * triangulation edge flips but operates in 3D Minkowski space.
 *
 * The operation rewires the topology of both triangles and updates their adjacency
 * information, followed by recalculating normals and distances.
 *
 * @param simplex The expanding simplex
 * @param triangleA First triangle sharing the edge (edge 0 is the one to rotate)
 * @param triangleAIndex Index of triangleA
 * @param heapIndex Heap position of triangleA
 */
void expandingSimplexRotateEdge(struct ExpandingSimplex* simplex, struct SimplexTriangle* triangleA, int triangleAIndex, int heapIndex) {
    // new triangles are setup so the edge to rotate is the first edge
    unsigned char triangleBIndex = triangleA->indexData.adjacentFaces[0];

    struct SimplexTriangle* triangleB = &simplex->triangles[triangleBIndex];

    unsigned char relativeIndex0 = triangleA->indexData.oppositePoints[0];
    unsigned char relativeIndex1 = NEXT_FACE(relativeIndex0);
    unsigned char relativeIndex2 = NEXT_FACE(relativeIndex1);

    // Rewire adjacency pointers
    triangleA->indexData.adjacentFaces[0] = triangleB->indexData.adjacentFaces[relativeIndex2];
    triangleB->indexData.adjacentFaces[relativeIndex1] = triangleA->indexData.adjacentFaces[1];
    triangleA->indexData.adjacentFaces[1] = triangleBIndex;
    triangleB->indexData.adjacentFaces[relativeIndex2] = triangleAIndex;

    // Rewire vertex indices
    triangleA->indexData.indices[1] = triangleB->indexData.indices[relativeIndex0];
    triangleB->indexData.indices[relativeIndex2] = triangleA->indexData.indices[2];

    // Rewire opposite point tracking
    triangleA->indexData.oppositePoints[0] = triangleB->indexData.oppositePoints[relativeIndex2];
    triangleB->indexData.oppositePoints[relativeIndex1] = triangleA->indexData.oppositePoints[1];
    triangleA->indexData.oppositePoints[1] = relativeIndex1;
    triangleB->indexData.oppositePoints[relativeIndex2] = 0;

    // Update back-references from triangles adjacent to the rotated edge
    struct SimplexTriangle* adjTriangle = &simplex->triangles[triangleA->indexData.adjacentFaces[0]];
    unsigned char adjIndex = NEXT_FACE(triangleA->indexData.oppositePoints[0]);
    adjTriangle->indexData.adjacentFaces[adjIndex] = triangleAIndex;
    adjTriangle->indexData.oppositePoints[adjIndex] = 2;

    adjTriangle = &simplex->triangles[triangleB->indexData.adjacentFaces[relativeIndex1]];
    adjIndex = NEXT_FACE(triangleB->indexData.oppositePoints[relativeIndex1]);
    adjTriangle->indexData.adjacentFaces[adjIndex] = triangleBIndex;
    adjTriangle->indexData.oppositePoints[adjIndex] = relativeIndex0;

    // Recalculate geometry for both affected triangles
    expandingSimplexTriangleInitNormal(simplex, triangleA);
    if (!(simplex->flags & SimplexFlagsSkipDistance)) {
        expandingSimplexTriangleDetermineDistance(simplex, triangleA);
        expandingSimplexFixHeap(simplex, heapIndex);
    }

    expandingSimplexTriangleInitNormal(simplex, triangleB);
    if (!(simplex->flags & SimplexFlagsSkipDistance)) {
        expandingSimplexTriangleDetermineDistance(simplex, triangleB);
        expandingSimplexFixHeap(simplex, simplex->triangleToHeapIndex[triangleBIndex]);
    }
}

/**
 * @brief Tests if an edge needs rotation and performs it if necessary.
 *
 * Checks if the opposite vertex from the adjacent triangle violates convexity
 * (lies above the current triangle's plane). If so, rotates the edge to restore
 * the convex hull property.
 *
 * @param simplex The expanding simplex
 * @param triangleIndex Triangle to check
 * @param heapIndex Heap position of the triangle
 */
void expandingSimplexTriangleCheckRotate(struct ExpandingSimplex* simplex, int triangleIndex, int heapIndex) {
    struct SimplexTriangle* triangle = &simplex->triangles[triangleIndex];
    struct SimplexTriangle* adjacent = &simplex->triangles[triangle->indexData.adjacentFaces[0]];
    Vector3* oppositePoint = &simplex->points[adjacent->indexData.indices[triangle->indexData.oppositePoints[0]]];

    Vector3* firstPoint = &simplex->points[triangle->indexData.indices[0]];

    Vector3 offset;
    vector3Sub(oppositePoint, firstPoint, &offset);

    // Convexity test: if opposite point is above the plane, edge must be rotated
    if (vector3Dot(&offset, &triangle->normal) > 0.0f) {
        expandingSimplexRotateEdge(simplex, triangle, triangleIndex, heapIndex);
    } else if (!(simplex->flags & SimplexFlagsSkipDistance)) {
        expandingSimplexTriangleDetermineDistance(simplex, triangle);
        expandingSimplexFixHeap(simplex, heapIndex);
    }
}

/**
 * @brief Initializes a triangle with topology data and computes its normal.
 *
 * @param simplex The expanding simplex
 * @param indexData Vertex indices and adjacency information
 * @param triangle Triangle to initialize
 */
void expandingSimplexTriangleInit(struct ExpandingSimplex* simplex, union SimplexTriangleIndexData* indexData, struct SimplexTriangle* triangle) {
    triangle->indexData = *indexData;
    expandingSimplexTriangleInitNormal(simplex, triangle);
}

/**
 * @brief Adds a new triangle to the expanding simplex.
 *
 * Initializes the triangle, calculates its distance, and inserts it into the min-heap.
 * The heap maintains O(1) access to the closest face, which is critical for EPA efficiency.
 *
 * @param simplex The expanding simplex
 * @param data Triangle topology data
 */
void expandingSimplexAddTriangle(struct ExpandingSimplex* simplex, union SimplexTriangleIndexData* data) {
    if (simplex->triangleCount == EPA_MAX_SIMPLEX_TRIANGLES) {
        return;
    }

    short result = simplex->triangleCount;
    expandingSimplexTriangleInit(simplex, data, &simplex->triangles[result]);
    ++simplex->triangleCount;

    if (simplex->flags & SimplexFlagsSkipDistance) {
        return;
    }

    expandingSimplexTriangleDetermineDistance(simplex, &simplex->triangles[result]);

    simplex->triangleHeap[result] = result;
    simplex->triangleToHeapIndex[result] = result;
    expandingSimplexSiftDownHeap(simplex, result);
}

/**
 * @brief Returns the triangle closest to the origin.
 *
 * The min-heap guarantees this is always at index 0, providing O(1) access.
 *
 * @param simplex The expanding simplex
 * @return Pointer to the closest triangle
 */
static inline struct SimplexTriangle* expandingSimplexClosestFace(struct ExpandingSimplex* simplex) {
    return &simplex->triangles[simplex->triangleHeap[0]];
}

// Initial topology for a tetrahedron (4 triangular faces)
union SimplexTriangleIndexData gInitialSimplexIndexData[] = {
    {{{0, 1, 2}, {3, 1, 2}, {2, 2, 2}}},
    {{{2, 1, 3}, {0, 3, 2}, {0, 1, 0}}},
    {{{0, 2, 3}, {0, 1, 3}, {1, 1, 0}}},
    {{{1, 0, 3}, {0, 2, 1}, {2, 1, 0}}},
};

/**
 * @brief Initializes the expanding simplex from a GJK simplex.
 *
 * Converts the 4-point GJK simplex (tetrahedron) into a polytope representation
 * with explicit triangle faces and adjacency information. This forms the initial
 * structure that EPA will expand.
 *
 * @param expandingSimplex The expanding simplex to initialize
 * @param simplex GJK simplex containing the origin (must have 4 points)
 * @param flags Initialization flags (e.g., SimplexFlagsSkipDistance for swept mode)
 */
void expandingSimplexInit(struct ExpandingSimplex* expandingSimplex, struct Simplex* simplex, int flags) {
    assert(simplex->nPoints == 4);

    expandingSimplex->triangleCount = 0;
    expandingSimplex->pointCount = 0;
    expandingSimplex->flags = flags;

    for (int i = 0; i < 4; ++i) {
        expandingSimplexAddPoint(expandingSimplex, &simplex->objectAPoint[i], &simplex->points[i]);
    }

    for (int i = 0; i < 4; ++i) {
        expandingSimplexAddTriangle(expandingSimplex, &gInitialSimplexIndexData[i]);
    }
}

/**
 * @brief Expands the polytope by replacing a face with three new faces.
 *
 * This is the core EPA expansion operation. The closest face to the origin is removed
 * and replaced with three new faces connecting the face's edges to the new support point.
 * This grows the polytope closer to the true Minkowski difference boundary.
 *
 * The operation maintains all adjacency relationships and checks for edge rotations
 * to preserve convexity.
 *
 * @param expandingSimplex The expanding simplex
 * @param newPointIndex Index of the new support point to add
 * @param faceToRemoveIndex Index of the face to replace (typically the closest face)
 */
void expandingSimplexExpand(struct ExpandingSimplex* expandingSimplex, int newPointIndex, int faceToRemoveIndex) {
    if (newPointIndex == -1) {
        return;
    }

    struct SimplexTriangle* faceToRemove = &expandingSimplex->triangles[faceToRemoveIndex];
    union SimplexTriangleIndexData existing = faceToRemove->indexData;

    // One face is replaced (reusing the index), two faces are added
    unsigned char triangleIndices[3];
    triangleIndices[0] = faceToRemoveIndex;
    triangleIndices[1] = expandingSimplex->triangleCount;
    triangleIndices[2] = expandingSimplex->triangleCount + 1;

    // Build three new faces, one for each edge of the removed face
    for (int i = 0; i < 3; ++i) {
        union SimplexTriangleIndexData next;
        unsigned char nextFace = NEXT_FACE(i);
        unsigned char nextNextFace = NEXT_FACE(nextFace);
        // Each new face is a triangle from an edge of the old face to the new point
        next.indices[0] = existing.indices[i];
        next.indices[1] = existing.indices[nextFace];
        next.indices[2] = newPointIndex;

        // Edge 0 connects to the old face's neighbor, edges 1 and 2 connect to the other new faces
        next.adjacentFaces[0] = existing.adjacentFaces[i];
        next.adjacentFaces[1] = triangleIndices[nextFace];
        next.adjacentFaces[2] = triangleIndices[nextNextFace];

        next.oppositePoints[0] = existing.oppositePoints[i];
        next.oppositePoints[1] = 1;
        next.oppositePoints[2] = 0;

        // Update the external neighbor to point back to this new face
        struct SimplexTriangle* otherTriangle = &expandingSimplex->triangles[existing.adjacentFaces[i]];
        unsigned char backReferenceIndex = NEXT_FACE(existing.oppositePoints[i]);
        otherTriangle->indexData.adjacentFaces[backReferenceIndex] = triangleIndices[i];
        otherTriangle->indexData.oppositePoints[backReferenceIndex] = 2;

        expandingSimplexTriangleInit(expandingSimplex, &next, &expandingSimplex->triangles[triangleIndices[i]]);
    }

    // Check if any edges need rotation to maintain convexity
    for (int i = 0; i < 3; ++i) {
        unsigned char triangleIndex = triangleIndices[i];

        if (i != 0) {
            expandingSimplex->triangleHeap[triangleIndex] = triangleIndex;
            expandingSimplex->triangleToHeapIndex[triangleIndex] = triangleIndex;  // initialize inverse mapping
            ++expandingSimplex->triangleCount;
        }

        expandingSimplexTriangleCheckRotate(expandingSimplex, triangleIndex, i == 0 ? 0 : triangleIndex);
    }
}

/**
 * @brief Calculates contact points from the closest face on the polytope.
 *
 * Uses barycentric coordinates to interpolate the exact contact point on object A's surface.
 * The contact point on B is then derived from A's point plus the penetration along the normal.
 *
 * @param simplex The expanding simplex
 * @param closestFace The face closest to the origin (defines the contact plane)
 * @param planePos Position on the face plane closest to the origin
 * @param result Output structure to store contact information
 */
void epaCalculateContact(struct ExpandingSimplex* simplex, struct SimplexTriangle* closestFace, Vector3* planePos, struct EpaResult* result) {
    Vector3 baryCoords;

    calculateBarycentricCoords(
        &simplex->points[closestFace->indexData.indices[0]],
        &simplex->points[closestFace->indexData.indices[1]],
        &simplex->points[closestFace->indexData.indices[2]],
        planePos,
        &baryCoords
    );

    evaluateBarycentricCoords(
        &simplex->aPoints[closestFace->indexData.indices[0]],
        &simplex->aPoints[closestFace->indexData.indices[1]],
        &simplex->aPoints[closestFace->indexData.indices[2]],
        &baryCoords,
        &result->contactA
    );

    vector3AddScaled(&result->contactA, &result->normal, result->penetration, &result->contactB);
}


bool epaSolve(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, struct EpaResult* result) {
    struct ExpandingSimplex simplex;
    expandingSimplexInit(&simplex, startingSimplex, 0);
    struct SimplexTriangle* closestFace = 0;
    float projection = 0.0f;

    for (int i = 0; i < EPA_MAX_ITERATIONS; ++i) {
        Vector3 reverseNormal;

        closestFace = expandingSimplexClosestFace(&simplex);

        short nextIndex = simplex.pointCount;

        Vector3* aPoint = &simplex.aPoints[nextIndex];
        Vector3 bPoint;

        // Get support point in the direction of the closest face's normal
        objectASupport(objectA, &closestFace->normal, aPoint);
        vector3Negate(&closestFace->normal, &reverseNormal);
        objectBSum(objectB, &reverseNormal, &bPoint);

        vector3Sub(aPoint, &bPoint, &simplex.points[nextIndex]);

        projection = vector3Dot(&simplex.points[nextIndex], &closestFace->normal);

        // Convergence test: if new support point doesn't significantly extend the polytope,
        // we've reached the Minkowski difference boundary
        if ((projection - closestFace->distanceToOrigin) < 0.001f) {
            break;
        }

        ++simplex.pointCount;
        expandingSimplexExpand(&simplex, nextIndex, simplex.triangleHeap[0]);
    }

    if (closestFace) {
        result->normal = closestFace->normal;
        vector3Negate(&result->normal, &result->normal);
        result->penetration = projection;
        Vector3 planePos;
        vector3Scale(&closestFace->normal, &planePos, closestFace->distanceToOrigin);
        epaCalculateContact(&simplex, closestFace, &planePos, result);

        return true;
    }
    
    return false;
}

#define MAX_SWEPT_ITERATIONS    15

/**
 * @brief Finds the face of the polytope that the sweep direction first encounters.
 *
 * For swept collision detection (continuous collision), we need to find where a moving
 * object first touches a stationary one. This walks the polytope surface from a starting
 * face, following edges in the direction opposite to the sweep, until finding the "frontmost"
 * face relative to the sweep direction.
 *
 * @param simplex The expanding simplex
 * @param direction Sweep direction vector
 * @param startTriangleIndex In/out: triangle index, updated to the found face
 * @param startFaceEdge In/out: edge index, updated during traversal
 */
void epaSweptFindFace(struct ExpandingSimplex* simplex, Vector3* direction, int* startTriangleIndex, int* startFaceEdge) {
    unsigned char currentFace = NEXT_FACE(*startFaceEdge);

    int i = 0;
    int loopCheck = 3;  // After checking all 3 edges without moving, we've found the face

    while (loopCheck > 0 && i < MAX_SWEPT_ITERATIONS) {
        unsigned char nextFace = NEXT_FACE(currentFace);

        struct SimplexTriangle* triangle = &simplex->triangles[*startTriangleIndex];

        // Compute edge normal (perpendicular to the edge in the face's plane)
        Vector3 normal;
        vector3Cross(
            &simplex->points[triangle->indexData.indices[currentFace]],
            &simplex->points[triangle->indexData.indices[nextFace]],
            &normal
        );

        // If direction points through this edge, move to the adjacent face
        if (vector3Dot(&normal, direction) < 0) {
            *startTriangleIndex = triangle->indexData.adjacentFaces[currentFace];
            *startFaceEdge = NEXT_FACE(triangle->indexData.oppositePoints[currentFace]);
            nextFace = NEXT_FACE(*startFaceEdge);
            loopCheck = 3;  // Reset counter since we moved to a new face
        }

        currentFace = nextFace;
        ++i;
        --loopCheck;
    }
}


bool epaSolveSwept(struct Simplex* startingSimplex, void* objectA, gjk_support_function objectASupport, void* objectB, gjk_support_function objectBSum, Vector3* bStart, Vector3* bEnd, struct EpaResult* result) {
    struct ExpandingSimplex simplex;
    expandingSimplexInit(&simplex, startingSimplex, SimplexFlagsSkipDistance);
    struct SimplexTriangle* closestFace = 0;
    float projection = 0.0f;
    int currentTriangle = 0;
    int currentEdge = 0;
    Vector3 raycastDir;
    vector3Sub(bStart, bEnd, &raycastDir);  // Direction from end to start (backtracking the sweep)

    for (int i = 0; i < EPA_MAX_ITERATIONS; ++i) {
        Vector3 reverseNormal;

        // Find the face that aligns with the sweep direction
        epaSweptFindFace(&simplex, &raycastDir, &currentTriangle, &currentEdge);
        closestFace = &simplex.triangles[currentTriangle];

        short nextIndex = simplex.pointCount;

        Vector3* aPoint = &simplex.aPoints[nextIndex];
        Vector3 bPoint;

        objectASupport(objectA, &closestFace->normal, aPoint);
        vector3Negate(&closestFace->normal, &reverseNormal);
        objectBSum(objectB, &reverseNormal, &bPoint);

        vector3Sub(aPoint, &bPoint, &simplex.points[nextIndex]);

        projection = vector3Dot(&simplex.points[nextIndex], &closestFace->normal);
        closestFace->distanceToOrigin = vector3Dot(&simplex.points[closestFace->indexData.indices[0]], &closestFace->normal);

        // Convergence test similar to standard EPA
        if ((projection - closestFace->distanceToOrigin) < 0.001f) {
            break;
        }

        ++simplex.pointCount;
        expandingSimplexExpand(&simplex, nextIndex, currentTriangle);
    }

    if (closestFace) {
        vector3Normalize(&raycastDir, &raycastDir);
        vector3Normalize(&closestFace->normal, &result->normal);

        // Create a plane from the contact face
        Plane facePlane;
        planeInitWithNormalAndPoint(&facePlane, &result->normal, &simplex.points[closestFace->indexData.indices[0]]);

        // Raycast from the origin along the sweep direction to find time of impact
        float distance;
        if (!planeRayIntersection(&facePlane, &gZeroVec, &raycastDir, &distance)) {
            return false;
        }

        // Add small epsilon to prevent numerical precision from causing slight overlap
        distance += 0.001f;

        result->penetration = 0;  // Swept collision results in contact, not penetration

        Vector3 planePos;
        vector3Scale(&raycastDir, &planePos, distance);
        float moveOffset = vector3DistSqrd(bStart, bEnd);

        // Verify collision occurs within the sweep range
        if (distance * distance >= moveOffset + 0.01f) {
            return false;
        }

        // Update bEnd to the position at first contact
        vector3Add(bEnd, &planePos, bEnd);
        epaCalculateContact(&simplex, closestFace, &planePos, result);
    }

    return true;
}