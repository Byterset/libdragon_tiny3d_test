#ifndef __AABB_TREE_H__
#define __AABB_TREE_H__

#include <stdint.h>
#include "../math/aabb.h"
#include "../math/vector3.h"
#include <assert.h>

#define AABB_TREE_NULL_NODE -1
#define AABB_TREE_DISPLACEMENT_MULTIPLIER 10.0f //this will multiply the expansion of the AABB of a Node according to how much it moved
#define AABB_TREE_NODE_BOUNDS_MARGIN 1.2f //this will be added to the bounds of a Node AABB so minor changes might not trigger a Node Movement
#define AABB_TREE_NODE_QUERY_STACK_SIZE 256

// Number representation of a node in the tree
typedef int16_t node_proxy;

/// @brief Provides a stack-like struct to push and pop nodes to/from
typedef struct node_stack {
    node_proxy stack[AABB_TREE_NODE_QUERY_STACK_SIZE];
    int top;
} node_stack;

/// @brief Push a node on an existing stack
/// @param s 
/// @param node 
static inline void node_stack_push(node_stack* s, node_proxy node) {
    s->stack[s->top++] = node;
};

/// @brief Pop a node from an existing stack
/// @param s 
/// @return 
static inline node_proxy node_stack_pop(node_stack* s) {
    return s->stack[--s->top];
};


/// @brief A node in the AABB_tree structure. Holds the bounds of the node, node_proxies to the parent, 
/// left and right children, the next node and a pointer to the collider data contained within.
typedef struct AABB_tree_node {
    AABB bounds; /*The bounds of the Node in World Space*/
    node_proxy _parent;
    node_proxy _left;
    node_proxy _right;
    node_proxy _next;
    void* data;
} AABB_tree_node;


/// @brief The AABB_tree holds the root node, the number of nodes in the tree, the capacity of the tree,
/// a free list of nodes, and an array of nodes.
typedef struct AABB_tree{
    node_proxy root;
    int16_t _nodeCount;
    int16_t _nodeCapacity;
    node_proxy _freeList;
    AABB_tree_node* nodes;

} AABB_tree;


/// @brief Set up an AABB_tree with an initial capacity
/// @param tree 
/// @param nodeCapacity 
void AABB_tree_init(AABB_tree* tree, int nodeCapacity);


/// @brief free the memory allocated for a tree
/// @param tree 
void AABB_tree_free(AABB_tree *tree);


/// @brief Query if the given node is a leaf node
/// @param node 
/// @return true if the node is a leaf, false otherwise
static inline bool AABB_tree_node_isLeaf(AABB_tree_node *node)
{
    return node->_left == AABB_TREE_NULL_NODE;
};

node_proxy AABB_tree_allocate_node(AABB_tree *tree);

void AABB_tree_free_node(AABB_tree *tree, node_proxy node);

/// @brief allocate & create a new node in the AABB_tree with the given Bounds and arbitrary data
/// @param tree 
/// @param bounds 
/// @param data 
/// @return the node proxy of the new node
node_proxy AABB_tree_create_node(AABB_tree *tree, AABB bounds, void* data);


/// @brief Moves a node within an AABB tree and updates its AABB.
///
/// The displacement vector is used to enlarge the node bounds by anticipated future movement and reduce the number of leaf insertions.
/// To avoid unnecessary tree updates, the node is only moved if the new AABB does not fit within the old AABB.
/// To disable the displacement feature, set the displacement vector to zero.
/// @param tree The AABB tree.
/// @param node The index of the node to move.
/// @param aabb The new AABB for the node.
/// @param displacement The displacement vector to move the node. Difference between the old and new position
/// @return Returns TRUE if the node was moved, FALSE otherwise.
bool AABB_tree_move_node(AABB_tree *tree, node_proxy node, AABB aabb, Vector3 *displacement);

void AABB_tree_rotate_node(AABB_tree *tree, node_proxy node);


/// @brief Tests if the bounds of two nodes of a Tree overlap
/// @param tree 
/// @param a first node
/// @param b second node
/// @return TRUE if the given nodes bounds overlap, FALSE otherwise
bool AABB_tree_node_test_node_overlap(AABB_tree *tree, node_proxy a, node_proxy b);


/// @brief Returns the Bounds in World Space of a given Node
/// @param tree 
/// @param node 
/// @return Pointer to the nodes AABB bounds
AABB* AABB_tree_get_node_bounds(AABB_tree *tree, node_proxy node);

void AABB_tree_rebuild(AABB_tree *tree);

node_proxy AABB_tree_insert_leaf_node(AABB_tree *tree, node_proxy leaf);


/// @brief Removes a leaf from the tree structure and frees the parent/re-arranges the sibling accordingly
///
/// Will also free the node if the freeNode flag is set to true
/// @param tree 
/// @param leaf 
/// @param freeNode 
void AABB_tree_remove_leaf_node(AABB_tree *tree, node_proxy leaf, int freeNode);


/// @brief Return the Collider Data associated with the given Node
/// @param tree
/// @param node
/// @return the pointer to the arbitrary node data
void* AABB_tree_get_node_data(const AABB_tree *tree, node_proxy node);


/// @brief Query the AABB_tree for (leaf) nodes that overlap with a given AABB
/// @param tree BVH tree
/// @param query_box the AABB to query for
/// @param results the pre-initialized array of NodeProxies to store the results
/// @param result_count the amount of results found
/// @param aabbChecks the amount of AABB checks performed
/// @param max_results the maximum amount of results to find
/// @param skipRootCheck if the root node should be checked
void AABB_tree_query_bounds(const AABB_tree *tree, const AABB *query_box, node_proxy *results, int* result_count, int max_results);


/// @brief Query the AABB_tree for (leaf) nodes that contain a point
/// @param tree BVH tree
/// @param point the point to query for
/// @param results the pre-initialized array of NodeProxies to store the results
/// @param result_count the amount of results found
/// @param max_results the maximum amount of results to find
void AABB_tree_query_point(const AABB_tree *tree, const Vector3 point, node_proxy *results, int *result_count, int max_results);


/// @brief Query the AABB_tree for (leaf) nodes that are intersected by a given Ray
/// @param tree BVH tree
/// @param ray the ray to query for
/// @param results the pre-initialized array of NodeProxies to store the results
/// @param result_count the amount of results found
/// @param max_results the maximum amount of results to find
void AABB_tree_query_ray(const AABB_tree *tree, const raycast *ray, node_proxy *results, int *result_count, int max_results);


/// @brief Generic AABB_tree query function that provides a scaffold for traversing the tree and testing nodes.
///
/// Accepts a AABB_query_function and a matching context to test the node bounds against various things (eg test for AABB / Ray / Point)
/// @param tree AABB_tree to query
/// @param query_function the query function which handles the test of the node bounds vs the given context
/// @param ctx the context of the query, can be a Point / Ray / AABB depending on query function
/// @param results the pre-initialized array of NodeProxies to store the results
/// @param result_count the amount of results found
/// @param max_results the maximum amount of results to find
void AABB_tree_query_generic(const AABB_tree *tree, const AABB_query_function query_function, const void* ctx, node_proxy *results, int *result_count, int max_results);

#endif // __AABB_TREE_H__