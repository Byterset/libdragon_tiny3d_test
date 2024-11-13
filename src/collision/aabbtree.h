#ifndef _AABBTREE_H_
#define _AABBTREE_H_

#include <stdint.h>
#include "../math/aabb.h"
#include "../util/vec.h"
#include "../math/vector3.h"
#include <assert.h>

#define NULL_NODE -1
#define AABBTREE_DISPLACEMENT_MULTIPLIER 8.0f
#define AABBTREE_NODE_BOUNDS_MARGIN 1.0f

typedef int16_t NodeProxy;

/// @brief A node in the AABBTree structure. Holds the bounds of the node, the parent, 
/// left and right children, the next node, a flag to indicate if the node has moved, 
/// and a pointer to the collider data contained within.
typedef struct AABBTreeNode {
    AABB bounds; /*The bounds of the Node in World Space*/
    NodeProxy _parent;
    NodeProxy _left;
    NodeProxy _right;
    NodeProxy _next;
    void* data;
} AABBTreeNode;


/// @brief The AABBTree holds the root node, the number of nodes in the tree, the capacity of the tree,
/// a free list of nodes, and an array of nodes.
typedef struct AABBTree{
    NodeProxy root;
    int16_t _nodeCount;
    int16_t _nodeCapacity;
    NodeProxy _freeList;
    AABBTreeNode* nodes;

} AABBTree;


void AABBTree_create(AABBTree* tree, int nodeCapacity);

void AABBTree_free(AABBTree *tree);

int AABBTreeNode_isLeaf(AABBTreeNode *node);

NodeProxy AABBTree_allocateNode(AABBTree *tree);

void AABBTreeNode_freeNode(AABBTree *tree, NodeProxy node);

NodeProxy AABBTreeNode_createNode(AABBTree *tree, AABB bounds, void* data);

int AABBTree_moveNode(AABBTree *tree, NodeProxy node, AABB aabb, Vector3 *displacement);

void AABBTreeNode_rotateNode(AABBTree *tree, NodeProxy node);

int AABBTreeNode_testOverlap(AABBTree *tree, NodeProxy a, NodeProxy b);

AABB* AABBTreeNode_getBounds(AABBTree *tree, NodeProxy node);

void AABBTree_rebuild(AABBTree *tree);

NodeProxy AABBTree_insertLeaf(AABBTree *tree, NodeProxy leaf);

void AABBTree_removeLeaf(AABBTree *tree, NodeProxy leaf, int freeNode);

AABB AABBTree_getNodeAABB(AABBTree *tree, NodeProxy node);

void AABBTreeNode_clearMoved(AABBTree *tree, NodeProxy node);

int AABBTreeNode_wasMoved(AABBTree *tree, NodeProxy node);

void* AABBTreeNode_getData(AABBTree *tree, NodeProxy node);

void AABBTree_queryBounds(AABBTree *tree, AABB *query_box, NodeProxy *results, int* result_count, int max_results);

void AABBTree_queryPoint(AABBTree *tree, Vector3 point, NodeProxy *results, int *result_count, int max_results);

void AABBTree_queryRay(AABBTree *tree, struct RayCast *ray, NodeProxy *results, int *result_count, int max_results);

#endif // _AABBTREE_H_