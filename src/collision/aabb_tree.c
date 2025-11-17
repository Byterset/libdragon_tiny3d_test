#include "aabb_tree.h"
#include <stdbool.h>
#include <malloc.h>
#include "../math/mathf.h"

/// @brief Allocate Memory for an AABB_tree with an initial node capacity
/// @param tree
/// @param nodeCapacity 
/// @return 
void AABB_tree_init(AABB_tree* tree, int nodeCapacity)
{
    tree->_nodeCapacity = nodeCapacity;
    //allocate space for the number of possible nodes and initialize with 0
    tree->nodes = (AABB_tree_node *)calloc(nodeCapacity, sizeof(AABB_tree_node));
    assert(tree->nodes);

    tree->root = AABB_TREE_NULL_NODE;
    tree->_nodeCount = 0;

    int i;
    for (i = 0; i < nodeCapacity - 1; ++i)
    {
        tree->nodes[i]._next = i + 1;
        tree->nodes[i]._parent = i;
    }
    tree->nodes[nodeCapacity - 1]._next = AABB_TREE_NULL_NODE;
    tree->nodes[nodeCapacity - 1]._parent = nodeCapacity - 1;

    tree->_freeList = 0;
}

void AABB_tree_free(AABB_tree *tree)
{
    if (tree == NULL)
    {
        return;
    }

    // Free the nodes array.
    if (tree->nodes != NULL)
    {
        free(tree->nodes);
        tree->nodes = NULL;
        tree->_nodeCount = 0;
        tree->_nodeCapacity = 0;
        tree->root = AABB_TREE_NULL_NODE;
    }
}

node_proxy AABB_tree_allocate_node(AABB_tree *tree)
{
    if (tree->_freeList == AABB_TREE_NULL_NODE)
    {
        assert(tree->_nodeCount == tree->_nodeCapacity);
        tree->_nodeCapacity *= 2;
        AABB_tree_node *nodes_new = (AABB_tree_node *)realloc(tree->nodes, sizeof(AABB_tree_node) * tree->_nodeCapacity);
        tree->nodes = nodes_new;
        int i;
        for (i = tree->_nodeCount; i < tree->_nodeCapacity - 1; ++i)
        {
            tree->nodes[i]._next = i + 1;
            tree->nodes[i]._parent = i;
        }
        tree->nodes[tree->_nodeCapacity - 1]._next = AABB_TREE_NULL_NODE;
        tree->nodes[tree->_nodeCapacity - 1]._parent = tree->_nodeCapacity - 1;
        tree->_freeList = tree->_nodeCount;
    }

    node_proxy node = tree->_freeList;
    tree->_freeList = tree->nodes[node]._next;
    tree->nodes[node]._parent = AABB_TREE_NULL_NODE;
    tree->nodes[node]._left = AABB_TREE_NULL_NODE;
    tree->nodes[node]._right = AABB_TREE_NULL_NODE;
    tree->nodes[node].data = NULL;
    tree->_nodeCount++;
    return node;
}

void AABB_tree_free_node(AABB_tree *tree, node_proxy node)
{
    assert(0 <= node && node <= tree->_nodeCapacity);
    assert(0 < tree->_nodeCount);
    tree->nodes[node]._parent = node;
    tree->nodes[node]._next = tree->_freeList;
    tree->nodes[node].data = NULL;
    tree->_freeList = node;
    tree->_nodeCount--;
}


node_proxy AABB_tree_create_node(AABB_tree *tree, AABB bounds, void* data)
{
    node_proxy newNode = AABB_tree_allocate_node(tree);

    //fatten the aabb by a margin
    tree->nodes[newNode].bounds = bounds;
    Vector3 bounds_margin = {{AABB_TREE_NODE_BOUNDS_MARGIN, AABB_TREE_NODE_BOUNDS_MARGIN, AABB_TREE_NODE_BOUNDS_MARGIN}};
    vector3Add(&tree->nodes[newNode].bounds.max, &bounds_margin, &tree->nodes[newNode].bounds.max);
    vector3Sub(&tree->nodes[newNode].bounds.min, &bounds_margin, &tree->nodes[newNode].bounds.min);
    tree->nodes[newNode].data = data;
    // tree->nodes[newNode].moved = true;
    tree->nodes[newNode]._parent = AABB_TREE_NULL_NODE;

    AABB_tree_insert_leaf_node(tree, newNode);
    return newNode;
}


bool AABB_tree_move_node(AABB_tree *tree, node_proxy node, AABB aabb, Vector3 *displacement)
{
    assert(0 <= node && node < tree->_nodeCapacity);
    assert(AABB_tree_node_isLeaf(&tree->nodes[node])); // only leaves can move

    AABB *treeAABB = &tree->nodes[node].bounds;

    // if the nodes current AABB contains the new AABB, return since the node can stay in place
    if (AABBContainsAABB(treeAABB, &aabb))
    {
        return false;
    }

    // scale the displacement vector by a factor to enlarge the Node Bounds & accommodate for anticipated movement
    // potentially reduces the amount of moves/leaf inserts
    Vector3 scaled_displacement;
    vector3Scale(displacement, &scaled_displacement, AABB_TREE_DISPLACEMENT_MULTIPLIER);
    AABBExtendDirection(&aabb, &scaled_displacement, &aabb);

    //fatten the aabb to potentially reduce the number of moves/leaf inserts
    Vector3 bounds_margin = {{AABB_TREE_NODE_BOUNDS_MARGIN, AABB_TREE_NODE_BOUNDS_MARGIN, AABB_TREE_NODE_BOUNDS_MARGIN}};
    vector3AddToSelf(&aabb.max, &bounds_margin);
    vector3SubFromSelf(&aabb.min, &bounds_margin);

    AABB_tree_remove_leaf_node(tree, node, false);

    tree->nodes[node].bounds = aabb;

    AABB_tree_insert_leaf_node(tree, node);

    return true;
}

void AABB_tree_rotate_node(AABB_tree *tree, node_proxy node)
{
    if (AABB_tree_node_isLeaf(&tree->nodes[node]))
    {
        return;
    }

    node_proxy left = tree->nodes[node]._left;
    node_proxy right = tree->nodes[node]._right;

    float costDiffs[4];
    costDiffs[0] = costDiffs[1] = costDiffs[2] = costDiffs[3] = 0.0f;

    if (AABB_tree_node_isLeaf(&tree->nodes[left]) == false)
    {
        float area1 = AABBGetArea(tree->nodes[left].bounds);

        costDiffs[0] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[left]._left].bounds, &tree->nodes[right].bounds)) - area1;
        costDiffs[1] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[left]._right].bounds, &tree->nodes[right].bounds)) - area1;
    }

    if (AABB_tree_node_isLeaf(&tree->nodes[right]) == false)
    {
        float area2 = AABBGetArea(tree->nodes[right].bounds);

        costDiffs[2] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[right]._left].bounds, &tree->nodes[left].bounds)) - area2;
        costDiffs[3] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[right]._right].bounds, &tree->nodes[left].bounds)) - area2;
    }

    int bestDiffIndex = 0;
    int i;
    for (i = 1; i < 4; ++i)
    {
        if (costDiffs[i] < costDiffs[bestDiffIndex])
        {
            bestDiffIndex = i;
        }
    }

    // Rotate only if it reduces the suface area
    if (costDiffs[bestDiffIndex] >= 0.0f)
    {
        return;
    }

    switch (bestDiffIndex)
    {
    case 0:
    {
        // Swap(right, tree->nodes[left]._right);
        tree->nodes[tree->nodes[left]._right]._parent = node;
        tree->nodes[node]._right = tree->nodes[left]._right;

        tree->nodes[left]._right = right;
        tree->nodes[right]._parent = left;

        tree->nodes[left].bounds = AABBUnion(&tree->nodes[tree->nodes[left]._left].bounds, &tree->nodes[tree->nodes[left]._right].bounds);
    }
    break;
    case 1:
    {
        // Swap(right, tree->nodes[left].left);
        tree->nodes[tree->nodes[left]._left]._parent = node;
        tree->nodes[node]._right = tree->nodes[left]._left;

        tree->nodes[left]._left = right;
        tree->nodes[right]._parent = left;

        tree->nodes[left].bounds = AABBUnion(&tree->nodes[tree->nodes[left]._left].bounds, &tree->nodes[tree->nodes[left]._right].bounds);
    }
    break;
    case 2:
    {
        // Swap(left, tree->nodes[right]._right);
        tree->nodes[tree->nodes[right]._right]._parent = node;
        tree->nodes[node]._left = tree->nodes[right]._right;

        tree->nodes[right]._right = left;
        tree->nodes[left]._parent = right;

        tree->nodes[right].bounds = AABBUnion(&tree->nodes[tree->nodes[right]._left].bounds, &tree->nodes[tree->nodes[right]._right].bounds);
    }
    break;
    case 3:
    {
        // Swap(left, tree->nodes[right].left);
        tree->nodes[tree->nodes[right]._left]._parent = node;
        tree->nodes[node]._left = tree->nodes[right]._left;

        tree->nodes[right]._left = left;
        tree->nodes[left]._parent = right;

        tree->nodes[right].bounds = AABBUnion(&tree->nodes[tree->nodes[right]._left].bounds, &tree->nodes[tree->nodes[right]._right].bounds);
    }
    break;
    }
}


bool AABB_tree_node_test_node_overlap(AABB_tree *tree, node_proxy a, node_proxy b)
{

    assert(0 <= a && a < tree->_nodeCapacity);
    assert(0 <= b && b < tree->_nodeCapacity);
    
    return AABBHasOverlap(&tree->nodes[a].bounds, &tree->nodes[b].bounds);
}


AABB *AABB_tree_get_node_bounds(AABB_tree *tree, node_proxy node)
{
    assert(0 <= node && node < tree->_nodeCapacity);
    return &tree->nodes[node].bounds;
}

void AABB_tree_rebuild(AABB_tree *tree)
{
    node_proxy *leaves = (node_proxy *)malloc(sizeof(node_proxy) * tree->_nodeCount);
    int count = 0;
    int i;

    for (i = 0; i < tree->_nodeCapacity; i++)
    {
        if (tree->nodes[i]._parent == i)
        {
            continue;
        }

        if (AABB_tree_node_isLeaf(&tree->nodes[i]))
        {
            tree->nodes[i]._parent = AABB_TREE_NULL_NODE;
            leaves[count++] = i;
        }
        else
        {
            AABB_tree_free_node(tree, i);
        }
    }

    while (count > 1)
    {
        float minCost = FLT_MAX;
        int minI = -1;
        int minJ = -1;
        int i, j;

        for (i = 0; i < count; i++)
        {
            AABB aabbi = tree->nodes[leaves[i]].bounds;

            for (j = i + 1; j < count; j++)
            {
                AABB aabbj = tree->nodes[leaves[j]].bounds;
                AABB combined = AABBUnion(&aabbi, &aabbj);
                float cost = AABBGetArea(combined);

                if (cost < minCost)
                {
                    minI = i;
                    minJ = j;
                    minCost = cost;
                }
            }
        }

        node_proxy index1 = leaves[minI];
        node_proxy index2 = leaves[minJ];
        AABB_tree_node *left = tree->nodes + index1;
        AABB_tree_node *right = tree->nodes + index2;

        node_proxy parentIndex = AABB_tree_allocate_node(tree);
        AABB_tree_node *parent = tree->nodes + parentIndex;

        parent->_left = index1;
        parent->_right = index2;
        parent->bounds = AABBUnion(&left->bounds, &right->bounds);
        parent->_parent = AABB_TREE_NULL_NODE;

        left->_parent = parentIndex;
        right->_parent = parentIndex;

        leaves[minI] = parentIndex;

        leaves[minJ] = leaves[count - 1];
        --count;
    }

    tree->root = leaves[0];
    free(leaves);
}

node_proxy AABB_tree_insert_leaf_node(AABB_tree *tree, node_proxy leaf)
{
    assert(0 <= leaf && leaf < tree->_nodeCapacity);
    assert(AABB_tree_node_isLeaf(&tree->nodes[leaf]));
    
    if (tree->root == AABB_TREE_NULL_NODE)
    {
        tree->root = leaf;
        return leaf;
    }

    AABB aabb = tree->nodes[leaf].bounds;

    // Find best Sibling for new Leaf
    node_proxy bestSibling = tree->root;
    float bestCost = AABBGetArea(AABBUnion(&aabb, &tree->nodes[tree->root].bounds));

    struct Candidate
    {
        node_proxy node;
        float inheritedCost;
    };

    // initialize the stack with the root node
    struct Candidate stack[AABB_TREE_NODE_QUERY_STACK_SIZE];
    int stack_top = 0;
    stack[stack_top++] = (struct Candidate){tree->root, 0.0f};

    while (stack_top > 0)
    {
        struct Candidate currentCandidate = stack[--stack_top];
        node_proxy current = currentCandidate.node;
        float inheritedCost = currentCandidate.inheritedCost;
        AABB combined = AABBUnion(&aabb, &tree->nodes[current].bounds);
        float directCost = AABBGetArea(combined);

        float cost = directCost + inheritedCost;
        if (cost < bestCost)
        {
            bestCost = cost;
            bestSibling = current;
        }

        inheritedCost += directCost - AABBGetArea(tree->nodes[current].bounds);

        float lowerBoundCost = AABBGetArea(aabb) + inheritedCost;

        if (lowerBoundCost < bestCost)
        {
            if (AABB_tree_node_isLeaf(&tree->nodes[current]) == false)
            {
                stack[stack_top++] = (struct Candidate){tree->nodes[current]._left, inheritedCost};
                stack[stack_top++] = (struct Candidate){tree->nodes[current]._right, inheritedCost};
            }
        }
    }

    // create a new Parent
    node_proxy oldParent = tree->nodes[bestSibling]._parent;
    node_proxy newParent = AABB_tree_allocate_node(tree);
    tree->nodes[newParent].bounds = AABBUnion(&aabb, &tree->nodes[bestSibling].bounds);
    tree->nodes[newParent].data = NULL;
    tree->nodes[newParent]._parent = oldParent;

    // connect the new leaf and sibling to new parent
    tree->nodes[newParent]._left = leaf;
    tree->nodes[newParent]._right = bestSibling;
    tree->nodes[leaf]._parent = newParent;
    tree->nodes[bestSibling]._parent = newParent;

    if (oldParent != AABB_TREE_NULL_NODE)
    {
        if (tree->nodes[oldParent]._left == bestSibling)
        {
            tree->nodes[oldParent]._left = newParent;
        }
        else
        {
            tree->nodes[oldParent]._right = newParent;
        }
    }
    else
    {
        tree->root = newParent;
    }

    // walk back up the tree fixing heights and areas
    node_proxy ancestor = newParent;
    while (ancestor != AABB_TREE_NULL_NODE)
    {
        node_proxy left = tree->nodes[ancestor]._left;
        node_proxy right = tree->nodes[ancestor]._right;

        tree->nodes[ancestor].bounds = AABBUnion(&tree->nodes[left].bounds, &tree->nodes[right].bounds);
        AABB_tree_rotate_node(tree, ancestor);

        ancestor = tree->nodes[ancestor]._parent;
    }

    return leaf;
}


void AABB_tree_remove_leaf_node(AABB_tree *tree, node_proxy leaf, int freeNode)
{
    assert(0 <= leaf && leaf < tree->_nodeCapacity);
    assert(AABB_tree_node_isLeaf(&tree->nodes[leaf]));

    node_proxy parent = tree->nodes[leaf]._parent;
    if (parent == AABB_TREE_NULL_NODE) // node is root
    {
        assert(tree->root == leaf);
        tree->root = AABB_TREE_NULL_NODE;
        return;
    }

    node_proxy grandParent = tree->nodes[parent]._parent;
    node_proxy sibling;
    if (tree->nodes[parent]._left == leaf)
    {
        sibling = tree->nodes[parent]._right;
    }
    else
    {
        sibling = tree->nodes[parent]._left;
    }

    AABB_tree_free_node(tree, parent);

    if (grandParent != AABB_TREE_NULL_NODE) // node has grandparent
    {
        tree->nodes[sibling]._parent = grandParent;

        if (tree->nodes[grandParent]._left == parent)
        {
            tree->nodes[grandParent]._left = sibling;
        }
        else
        {
            tree->nodes[grandParent]._right = sibling;
        }

        node_proxy ancestor = grandParent;
        while (ancestor != AABB_TREE_NULL_NODE)
        {
            node_proxy left = tree->nodes[ancestor]._left;
            node_proxy right = tree->nodes[ancestor]._right;

            tree->nodes[ancestor].bounds = AABBUnion(&tree->nodes[left].bounds, &tree->nodes[right].bounds);

            AABB_tree_rotate_node(tree, ancestor);

            ancestor = tree->nodes[ancestor]._parent;
        }
    }
    else // node has no grandparent
    {
        tree->root = sibling;
        tree->nodes[sibling]._parent = AABB_TREE_NULL_NODE;
    }

    if(freeNode){
        AABB_tree_free_node(tree, leaf);
    }
}


void* AABB_tree_get_node_data(const AABB_tree *tree, node_proxy node)
{
    assert(0 <= node && node < tree->_nodeCapacity);
    return tree->nodes[node].data;
}


/// @brief Wrapper as point-aabb query function for generic tree query
/// @param bounds 
/// @param ctx void pointer to aabb struct
/// @return 
const bool _queryAABBOverlap(const AABB *bounds, const void *ctx) {
    AABB *query_box = (AABB*)ctx;
    return AABBHasOverlap(bounds, query_box);
}


void AABB_tree_query_bounds(const AABB_tree *tree, const AABB *query_box, node_proxy *results, int *result_count, int max_results)
{
    AABB_tree_query_generic(tree, _queryAABBOverlap, query_box, results, result_count, max_results);
}


/// @brief Wrapper as point-aabb query function for generic tree query
/// @param bounds 
/// @param ctx void pointer to vector3 struct
/// @return 
const bool _queryAABBContainsPoint(const AABB *bounds, const void *ctx) {
    Vector3 *point = (Vector3*)ctx;
    return AABBContainsPoint(bounds, point);
}


void AABB_tree_query_point(const AABB_tree *tree, const Vector3 point, node_proxy *results, int *result_count, int max_results)
{
    AABB_tree_query_generic(tree, _queryAABBContainsPoint, &point, results, result_count, max_results);
}

/// @brief Wrapper as ray-aabb query function for generic tree query
/// @param bounds 
/// @param ctx void pointer to raycast struct
/// @return 
const bool _queryAABBIntersectsRay(const AABB *bounds, const void *ctx) {
    raycast *ray = (raycast*)ctx;
    return AABBIntersectsRay(bounds, ray);
}



void AABB_tree_query_ray(const AABB_tree *tree, const raycast* ray, node_proxy *results, int *result_count, int max_results){
    AABB_tree_query_generic(tree, _queryAABBIntersectsRay, ray, results, result_count, max_results);
}


void AABB_tree_query_generic(
    const AABB_tree *tree,
    const AABB_query_function query_function, 
    const void* ctx, 
    node_proxy *results, 
    int *result_count, 
    int max_results)
{
    // return if the tree is empty
    if (tree->root == AABB_TREE_NULL_NODE)
    {
        return;
    }

    // initialize the stack with the root node
    node_stack stack = {.top = 1};
    stack.stack[0] = tree->root;

    AABB_tree_node *nodes = tree->nodes;
    int count = 0;

    while (stack.top > 0 && count < max_results)
    {
        node_proxy current = node_stack_pop(&stack);

        // if the point is not inside the bounds of the current node, continue and thus discard all child nodes
        AABB_tree_node *node = &nodes[current];
        if (!query_function(&node->bounds, ctx))
            continue;

        // Check if leaf using cached node
        if (AABB_tree_node_isLeaf(node))
        {
            results[count++] = current;
        }
        else
        {
            //guard for stack overflow
            if(stack.top + 2 > AABB_TREE_NODE_QUERY_STACK_SIZE) break;

            // Order matters for cache locality - push right first so left is processed first
            node_stack_push(&stack, node->_right);
            node_stack_push(&stack, node->_left);
        }
    }

    *result_count = count;
}