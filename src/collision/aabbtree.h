#ifndef _AABBTREE_H_
#define _AABBTREE_H_

#include <stdint.h>
#include "../math/aabb.h"
#include "../util/vec.h"
#include "../math/vector3.h"
#include <assert.h>

#define NULL_NODE -1
#define AABBTREE_DISPLACEMENT_MULTIPLIER 4.0f
#define AABBTREE_NODE_BOUNDS_MARGIN 1.0f

typedef int16_t NodeProxy;

/// @brief A node in the AABBTree structure. Holds the bounds of the node, the parent, 
/// left and right children, the next node, a flag to indicate if the node has moved, 
/// and a pointer to the collider data contained within.
typedef struct AABBTreeNode {
    struct AABB bounds; /*The bounds of the Node in World Space*/
    NodeProxy parent;
    NodeProxy left;
    NodeProxy right;
    NodeProxy next;
    // int moved;
    void* data;
} AABBTreeNode;


/// @brief The AABBTree holds the root node, the number of nodes in the tree, the capacity of the tree,
/// a free list of nodes, and an array of nodes.
typedef struct AABBTree{
    NodeProxy root;
    int16_t nodeCount;
    int16_t nodeCapacity;
    NodeProxy freeList;
    AABBTreeNode* nodes;

} AABBTree;


void AABBTree_create(AABBTree* tree, int nodeCapacity);

void AABBTree_free(AABBTree *tree);

int AABBTreeNode_isLeaf(AABBTreeNode *node);

NodeProxy AABBTree_allocateNode(AABBTree *tree);

void AABBTreeNode_freeNode(AABBTree *tree, NodeProxy node);

NodeProxy AABBTreeNode_createNode(AABBTree *tree, struct AABB bounds, void* data);

int AABBTree_moveNode(AABBTree *tree, NodeProxy node, struct AABB aabb, struct Vector3 *displacement);

void AABBTreeNode_rotateNode(AABBTree *tree, NodeProxy node);

void AABBTreeNode_removeNode(AABBTree *tree, NodeProxy node);

int AABBTreeNode_testOverlap(AABBTree *tree, NodeProxy a, NodeProxy b);

struct AABB* AABBTreeNode_getBounds(AABBTree *tree, NodeProxy node);

void AABBTree_rebuild(AABBTree *tree);

NodeProxy AABBTree_insertLeaf(AABBTree *tree, NodeProxy leaf);

void AABBTree_removeLeaf(AABBTree *tree, NodeProxy leaf);

struct AABB AABBTree_getNodeAABB(AABBTree *tree, NodeProxy node);

void AABBTreeNode_clearMoved(AABBTree *tree, NodeProxy node);

int AABBTreeNode_wasMoved(AABBTree *tree, NodeProxy node);

void* AABBTreeNode_getData(AABBTree *tree, NodeProxy node);

void AABBTree_queryBounds(AABBTree *tree, struct AABB *query_box, NodeProxy *results, int* result_count, int* aabbChecks, int max_results, int skipRootCheck);

void AABBTree_queryPoint(AABBTree *tree, struct Vector3 point, NodeProxy *results, int* result_count, int max_results);

#endif // _AABBTREE_H_