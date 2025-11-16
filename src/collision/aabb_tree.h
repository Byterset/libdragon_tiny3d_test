#ifndef __AABB_TREE_H__
#define __AABB_TREE_H__

#include <stdint.h>
#include "../math/aabb.h"
#include "../util/vec.h"
#include "../math/vector3.h"
#include <assert.h>

#define AABB_TREE_NULL_NODE -1
#define AABB_TREE_DISPLACEMENT_MULTIPLIER 10.0f //this will multiply the expansion of the AABB of a Node according to how much it moved
#define AABB_TREE_NODE_BOUNDS_MARGIN 1.2f //this will be added to the bounds of a Node AABB so minor changes might not trigger a Node Movement
#define AABB_TREE_NODE_QUERY_STACK_SIZE 256

typedef int16_t node_proxy;

typedef struct node_stack {
    node_proxy stack[AABB_TREE_NODE_QUERY_STACK_SIZE];
    int top;
} node_stack;

static inline void node_stack_push(node_stack* s, node_proxy node) {
    s->stack[s->top++] = node;
};

static inline node_proxy node_stack_pop(node_stack* s) {
    return s->stack[--s->top];
};


/// @brief A node in the AABB_tree structure. Holds the bounds of the node, the parent, 
/// left and right children, the next node, a flag to indicate if the node has moved, 
/// and a pointer to the collider data contained within.
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


void AABB_tree_init(AABB_tree* tree, int nodeCapacity);

void AABB_tree_free(AABB_tree *tree);

static inline int AABB_tree_node_isLeaf(AABB_tree_node *node)
{
    return node->_left == AABB_TREE_NULL_NODE;
};

node_proxy AABB_tree_allocate_node(AABB_tree *tree);

void AABB_tree_free_node(AABB_tree *tree, node_proxy node);

node_proxy AABB_tree_create_node(AABB_tree *tree, AABB bounds, void* data);

int AABB_tree_move_node(AABB_tree *tree, node_proxy node, AABB aabb, Vector3 *displacement);

void AABB_tree_rotate_node(AABB_tree *tree, node_proxy node);

int AABB_tree_node_test_node_overlap(AABB_tree *tree, node_proxy a, node_proxy b);

AABB* AABB_tree_get_node_bounds(AABB_tree *tree, node_proxy node);

void AABB_tree_rebuild(AABB_tree *tree);

node_proxy AABB_tree_insert_leaf_node(AABB_tree *tree, node_proxy leaf);

void AABB_tree_remove_leaf_node(AABB_tree *tree, node_proxy leaf, int freeNode);

void* AABB_tree_get_node_data(AABB_tree *tree, node_proxy node);

void AABB_tree_query_bounds(AABB_tree *tree, AABB *query_box, node_proxy *results, int* result_count, int max_results);

void AABB_tree_query_point(AABB_tree *tree, Vector3 point, node_proxy *results, int *result_count, int max_results);

void AABB_tree_query_ray(AABB_tree *tree, raycast *ray, node_proxy *results, int *result_count, int max_results);

void AABB_tree_query_generic(AABB_tree *tree, AABBQueryFunction query_function, void* ctx, node_proxy *results, int *result_count, int max_results);

#endif // __AABB_TREE_H__