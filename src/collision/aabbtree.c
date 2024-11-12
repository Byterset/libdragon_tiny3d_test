#include "aabbtree.h"
#include <stdbool.h>
#include <malloc.h>
#include "../math/mathf.h"

/// @brief Allocate Memory for an AABBTree with an initial node capacity
/// @param nodeCapacity 
/// @return 
void AABBTree_create(AABBTree* tree, int nodeCapacity)
{
    // AABBTree *tree = malloc(sizeof(AABBTree));
    tree->nodeCapacity = nodeCapacity;
    tree->nodes = (AABBTreeNode *)calloc(nodeCapacity, sizeof(AABBTreeNode));
    assert(tree->nodes);

    tree->root = NULL_NODE;
    tree->nodeCount = 0;

    int i;
    for (i = 0; i < nodeCapacity - 1; ++i)
    {
        tree->nodes[i].next = i + 1;
        tree->nodes[i].parent = i;
    }
    tree->nodes[nodeCapacity - 1].next = NULL_NODE;
    tree->nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

    tree->freeList = 0;

    // return tree;
}

void AABBTree_free(AABBTree *tree)
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
        tree->nodeCount = 0;
        tree->nodeCapacity = 0;
        tree->root = NULL_NODE;
    }
}

int AABBTreeNode_isLeaf(AABBTreeNode *node)
{
    return node->left == NULL_NODE;
}

NodeProxy AABBTree_allocateNode(AABBTree *tree)
{
    if (tree->freeList == NULL_NODE)
    {
        assert(tree->nodeCount == tree->nodeCapacity);
        tree->nodeCapacity += 20;
        AABBTreeNode *nodes_new = (AABBTreeNode *)realloc(tree->nodes, sizeof(AABBTreeNode) * tree->nodeCapacity);
        int totalSize = sizeof(AABBTreeNode) * tree->nodeCapacity;
        tree->nodes = nodes_new;
        int i;
        for (i = tree->nodeCount; i < tree->nodeCapacity - 1; ++i)
        {
            tree->nodes[i].next = i + 1;
            tree->nodes[i].parent = i;
        }
        tree->nodes[tree->nodeCapacity - 1].next = NULL_NODE;
        tree->nodes[tree->nodeCapacity - 1].parent = tree->nodeCapacity - 1;
        tree->freeList = tree->nodeCount;
    }

    NodeProxy node = tree->freeList;
    tree->freeList = tree->nodes[node].next;
    tree->nodes[node].parent = NULL_NODE;
    tree->nodes[node].left = NULL_NODE;
    tree->nodes[node].right = NULL_NODE;
    tree->nodes[node].data = NULL;
    // tree->nodes[node].moved = false;
    tree->nodeCount++;
    return node;
}

void AABBTreeNode_freeNode(AABBTree *tree, NodeProxy node)
{
    assert(0 <= node && node <= tree->nodeCapacity);
    assert(0 < tree->nodeCount);
    tree->nodes[node].parent = node;
    tree->nodes[node].next = tree->freeList;
    tree->nodes[node].data = NULL;
    tree->freeList = node;
    tree->nodeCount--;
}

/// @brief allocate & create a new node in the AABBTree with the given Bounds and arbitrary data
/// @param tree 
/// @param bounds 
/// @param data 
/// @return 
NodeProxy AABBTreeNode_createNode(AABBTree *tree, struct AABB bounds, void* data)
{
    NodeProxy newNode = AABBTree_allocateNode(tree);

    //fatten the aabb by a margin
    tree->nodes[newNode].bounds = bounds;
    struct Vector3 bounds_margin = {AABBTREE_NODE_BOUNDS_MARGIN, AABBTREE_NODE_BOUNDS_MARGIN, AABBTREE_NODE_BOUNDS_MARGIN};
    vector3Add(&tree->nodes[newNode].bounds.max, &bounds_margin, &tree->nodes[newNode].bounds.max);
    vector3Sub(&tree->nodes[newNode].bounds.min, &bounds_margin, &tree->nodes[newNode].bounds.min);
    tree->nodes[newNode].data = data;
    // tree->nodes[newNode].moved = true;
    tree->nodes[newNode].parent = NULL_NODE;

    AABBTree_insertLeaf(tree, newNode);
    return newNode;
}

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
int AABBTree_moveNode(AABBTree *tree, NodeProxy node, struct AABB aabb, struct Vector3 *displacement)
{
    assert(0 <= node && node < tree->nodeCapacity);
    assert(AABBTreeNode_isLeaf(&tree->nodes[node])); // only leaves can move

    struct AABB *treeAABB = &tree->nodes[node].bounds;

    // if the nodes current AABB contains the new AABB, return since the node can stay in place
    if (AABBContainsAABB(treeAABB, &aabb))
    {
        return false;
    }

    // scale the displacement vector by a factor to enlarge the Node Bounds & accommodate for anticipated movement
    // potentially reduces the amount of moves/leaf inserts

    if (displacement->x > 0.0f)
    {
        aabb.max.x += displacement->x * AABBTREE_DISPLACEMENT_MULTIPLIER;
    }
    else
    {
        aabb.min.x += displacement->x * AABBTREE_DISPLACEMENT_MULTIPLIER;
    }

    if (displacement->y > 0.0f)
    {
        aabb.max.y += displacement->y * AABBTREE_DISPLACEMENT_MULTIPLIER;
    }
    else
    {
        aabb.min.y += displacement->y * AABBTREE_DISPLACEMENT_MULTIPLIER;
    }

    if (displacement->z > 0.0f)
    {
        aabb.max.z += displacement->z * AABBTREE_DISPLACEMENT_MULTIPLIER;
    }
    else
    {
        aabb.min.z += displacement->z * AABBTREE_DISPLACEMENT_MULTIPLIER;
    }

    //fatten the aabb
    // struct Vector3 bounds_margin = {AABBTREE_NODE_BOUNDS_MARGIN, AABBTREE_NODE_BOUNDS_MARGIN, AABBTREE_NODE_BOUNDS_MARGIN};
    // vector3AddToSelf(&aabb.max, &bounds_margin);
    // vector3SubFromSelf(&aabb.min, &bounds_margin);

    AABBTree_removeLeaf(tree, node, false);

    tree->nodes[node].bounds = aabb;

    AABBTree_insertLeaf(tree, node);

    return true;
}

void AABBTreeNode_rotateNode(AABBTree *tree, NodeProxy node)
{
    if (AABBTreeNode_isLeaf(&tree->nodes[node]))
    {
        return;
    }

    NodeProxy left = tree->nodes[node].left;
    NodeProxy right = tree->nodes[node].right;

    float costDiffs[4];
    costDiffs[0] = costDiffs[1] = costDiffs[2] = costDiffs[3] = 0.0f;

    if (AABBTreeNode_isLeaf(&tree->nodes[left]) == false)
    {
        float area1 = AABBGetArea(tree->nodes[left].bounds);

        costDiffs[0] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[left].left].bounds, &tree->nodes[right].bounds)) - area1;
        costDiffs[1] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[left].right].bounds, &tree->nodes[right].bounds)) - area1;
    }

    if (AABBTreeNode_isLeaf(&tree->nodes[right]) == false)
    {
        float area2 = AABBGetArea(tree->nodes[right].bounds);

        costDiffs[2] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[right].left].bounds, &tree->nodes[left].bounds)) - area2;
        costDiffs[3] = AABBGetArea(AABBUnion(&tree->nodes[tree->nodes[right].right].bounds, &tree->nodes[left].bounds)) - area2;
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
        // Swap(right, tree->nodes[left].right);
        tree->nodes[tree->nodes[left].right].parent = node;
        tree->nodes[node].right = tree->nodes[left].right;

        tree->nodes[left].right = right;
        tree->nodes[right].parent = left;

        tree->nodes[left].bounds = AABBUnion(&tree->nodes[tree->nodes[left].left].bounds, &tree->nodes[tree->nodes[left].right].bounds);
    }
    break;
    case 1:
    {
        // Swap(right, tree->nodes[left].left);
        tree->nodes[tree->nodes[left].left].parent = node;
        tree->nodes[node].right = tree->nodes[left].left;

        tree->nodes[left].left = right;
        tree->nodes[right].parent = left;

        tree->nodes[left].bounds = AABBUnion(&tree->nodes[tree->nodes[left].left].bounds, &tree->nodes[tree->nodes[left].right].bounds);
    }
    break;
    case 2:
    {
        // Swap(left, tree->nodes[right].right);
        tree->nodes[tree->nodes[right].right].parent = node;
        tree->nodes[node].left = tree->nodes[right].right;

        tree->nodes[right].right = left;
        tree->nodes[left].parent = right;

        tree->nodes[right].bounds = AABBUnion(&tree->nodes[tree->nodes[right].left].bounds, &tree->nodes[tree->nodes[right].right].bounds);
    }
    break;
    case 3:
    {
        // Swap(left, tree->nodes[right].left);
        tree->nodes[tree->nodes[right].left].parent = node;
        tree->nodes[node].left = tree->nodes[right].left;

        tree->nodes[right].left = left;
        tree->nodes[left].parent = right;

        tree->nodes[right].bounds = AABBUnion(&tree->nodes[tree->nodes[right].left].bounds, &tree->nodes[tree->nodes[right].right].bounds);
    }
    break;
    }
}

/// @brief Tests if the bounds of two nodes of a Tree overlap
/// @param tree 
/// @param a first Node
/// @param b second Node
/// @return 
int AABBTreeNode_testOverlap(AABBTree *tree, NodeProxy a, NodeProxy b)
{

    assert(0 <= a && a < tree->nodeCapacity);
    assert(0 <= b && b < tree->nodeCapacity);
    
    return AABBHasOverlap(&tree->nodes[a].bounds, &tree->nodes[b].bounds);
}

/// @brief Returns the Bounds in World Space of a given Node
/// @param tree 
/// @param node 
/// @return 
struct AABB *AABBTreeNode_getBounds(AABBTree *tree, NodeProxy node)
{
    assert(0 <= node && node < tree->nodeCapacity);
    return &tree->nodes[node].bounds;
}

void AABBTree_rebuild(AABBTree *tree)
{
    NodeProxy *leaves = (NodeProxy *)malloc(sizeof(NodeProxy) * tree->nodeCount);
    int count = 0;
    int i;

    for (i = 0; i < tree->nodeCapacity; i++)
    {
        if (tree->nodes[i].parent == i)
        {
            continue;
        }

        if (AABBTreeNode_isLeaf(&tree->nodes[i]))
        {
            tree->nodes[i].parent = NULL_NODE;
            leaves[count++] = i;
        }
        else
        {
            AABBTreeNode_freeNode(tree, i);
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
            struct AABB aabbi = tree->nodes[leaves[i]].bounds;

            for (j = i + 1; j < count; j++)
            {
                struct AABB aabbj = tree->nodes[leaves[j]].bounds;
                struct AABB combined = AABBUnion(&aabbi, &aabbj);
                float cost = AABBGetArea(combined);

                if (cost < minCost)
                {
                    minI = i;
                    minJ = j;
                    minCost = cost;
                }
            }
        }

        NodeProxy index1 = leaves[minI];
        NodeProxy index2 = leaves[minJ];
        AABBTreeNode *left = tree->nodes + index1;
        AABBTreeNode *right = tree->nodes + index2;

        NodeProxy parentIndex = AABBTree_allocateNode(tree);
        AABBTreeNode *parent = tree->nodes + parentIndex;

        parent->left = index1;
        parent->right = index2;
        parent->bounds = AABBUnion(&left->bounds, &right->bounds);
        parent->parent = NULL_NODE;

        left->parent = parentIndex;
        right->parent = parentIndex;

        leaves[minI] = parentIndex;

        leaves[minJ] = leaves[count - 1];
        --count;
    }

    tree->root = leaves[0];
    free(leaves);
}

NodeProxy AABBTree_insertLeaf(AABBTree *tree, NodeProxy leaf)
{
    assert(0 <= leaf && leaf < tree->nodeCapacity);
    assert(AABBTreeNode_isLeaf(&tree->nodes[leaf]));
    
    if (tree->root == NULL_NODE)
    {
        tree->root = leaf;
        return leaf;
    }

    struct AABB aabb = tree->nodes[leaf].bounds;

    // Find best Sibling for new Leaf
    NodeProxy bestSibling = tree->root;
    float bestCost = AABBGetArea(AABBUnion(&aabb, &tree->nodes[tree->root].bounds));

    struct Candidate
    {
        NodeProxy node;
        float inheritedCost;
    };

    vec_t *stack = vec_with_capacity(256, sizeof(struct Candidate));
    vec_push(stack, &(struct Candidate){tree->root, 0.0f});

    while (stack->len != 0)
    {
        struct Candidate currentCandidate;
        vec_pop(stack, &currentCandidate);
        NodeProxy current = currentCandidate.node;
        float inheritedCost = currentCandidate.inheritedCost;
        struct AABB combined = AABBUnion(&aabb, &tree->nodes[current].bounds);
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
            if (AABBTreeNode_isLeaf(&tree->nodes[current]) == false)
            {
                vec_push(stack, &(struct Candidate){tree->nodes[current].left, inheritedCost});
                vec_push(stack, &(struct Candidate){tree->nodes[current].right, inheritedCost});
            }
        }
    }
    vec_drop(stack);

    // create a new Parent
    NodeProxy oldParent = tree->nodes[bestSibling].parent;
    NodeProxy newParent = AABBTree_allocateNode(tree);
    tree->nodes[newParent].bounds = AABBUnion(&aabb, &tree->nodes[bestSibling].bounds);
    tree->nodes[newParent].data = NULL;
    tree->nodes[newParent].parent = oldParent;

    // connect the new leaf and sibling to new parent
    tree->nodes[newParent].left = leaf;
    tree->nodes[newParent].right = bestSibling;
    tree->nodes[leaf].parent = newParent;
    tree->nodes[bestSibling].parent = newParent;

    if (oldParent != NULL_NODE)
    {
        if (tree->nodes[oldParent].left == bestSibling)
        {
            tree->nodes[oldParent].left = newParent;
        }
        else
        {
            tree->nodes[oldParent].right = newParent;
        }
    }
    else
    {
        tree->root = newParent;
    }

    // walk back up the tree fixing heights and areas
    NodeProxy ancestor = newParent;
    while (ancestor != NULL_NODE)
    {
        NodeProxy left = tree->nodes[ancestor].left;
        NodeProxy right = tree->nodes[ancestor].right;

        tree->nodes[ancestor].bounds = AABBUnion(&tree->nodes[left].bounds, &tree->nodes[right].bounds);
        AABBTreeNode_rotateNode(tree, ancestor);

        ancestor = tree->nodes[ancestor].parent;
    }

    return leaf;
}

/// @brief Removes a leaf from the tree structure and frees the parent/re-arranges the sibling accordingly
///
/// Will also free the node if the freeNode flag is set to true
/// @param tree 
/// @param leaf 
/// @param freeNode 
void AABBTree_removeLeaf(AABBTree *tree, NodeProxy leaf, bool freeNode)
{
    assert(0 <= leaf && leaf < tree->nodeCapacity);
    assert(AABBTreeNode_isLeaf(&tree->nodes[leaf]));

    NodeProxy parent = tree->nodes[leaf].parent;
    if (parent == NULL_NODE) // node is root
    {
        assert(tree->root == leaf);
        tree->root = NULL_NODE;
        return;
    }

    NodeProxy grandParent = tree->nodes[parent].parent;
    NodeProxy sibling;
    if (tree->nodes[parent].left == leaf)
    {
        sibling = tree->nodes[parent].right;
    }
    else
    {
        sibling = tree->nodes[parent].left;
    }

    AABBTreeNode_freeNode(tree, parent);

    if (grandParent != NULL_NODE) // node has grandparent
    {
        tree->nodes[sibling].parent = grandParent;

        if (tree->nodes[grandParent].left == parent)
        {
            tree->nodes[grandParent].left = sibling;
        }
        else
        {
            tree->nodes[grandParent].right = sibling;
        }

        NodeProxy ancestor = grandParent;
        while (ancestor != NULL_NODE)
        {
            NodeProxy left = tree->nodes[ancestor].left;
            NodeProxy right = tree->nodes[ancestor].right;

            tree->nodes[ancestor].bounds = AABBUnion(&tree->nodes[left].bounds, &tree->nodes[right].bounds);

            AABBTreeNode_rotateNode(tree, ancestor);

            ancestor = tree->nodes[ancestor].parent;
        }
    }
    else // node has no grandparent
    {
        tree->root = sibling;
        tree->nodes[sibling].parent = NULL_NODE;
    }

    if(freeNode){
        AABBTreeNode_freeNode(tree, leaf);
    }
}

struct AABB AABBTree_getNodeAABB(AABBTree *tree, NodeProxy node)
{
    assert(0 <= node && node < tree->nodeCapacity);
    return tree->nodes[node].bounds;
}

// void AABBTreeNode_clearMoved(AABBTree *tree, NodeProxy node)
// {
//     assert(0 <= node && node < tree->nodeCapacity);
//     tree->nodes[node].moved = false;
// }

/// @brief Check if the given node was moved
/// @param tree
/// @param node
/// @return
// int AABBTreeNode_wasMoved(AABBTree *tree, NodeProxy node)
// {
//     assert(0 <= node && node < tree->nodeCapacity);
//     return tree->nodes[node].moved;
// }

/// @brief Return the Collider Data associated with the given Node
/// @param tree
/// @param node
/// @return
void* AABBTreeNode_getData(AABBTree *tree, NodeProxy node)
{
    assert(0 <= node && node < tree->nodeCapacity);
    return tree->nodes[node].data;
}

/// @brief Query the AABBTree for (leaf) nodes that overlap with a given AABB
/// @param tree BVH tree
/// @param query_box the AABB to query for
/// @param results the pre-initialized array of NodeProxies to store the results
/// @param result_count the amount of results found
/// @param aabbChecks the amount of AABB checks performed
/// @param max_results the maximum amount of results to find
/// @param skipRootCheck if the root node should be checked
void AABBTree_queryBounds(AABBTree *tree, struct AABB *query_box, NodeProxy *results, int *result_count, int max_results)
{
    // return if the tree is empty
    if (tree->root == NULL_NODE)
    {
        return;
    }

    // initialize the stack with the root node
    NodeProxy stack[256];
    int stackSize = 0;
    stack[stackSize++] = tree->root;

    NodeProxy current;

    while (stackSize > 0)
    {
        NodeProxy current = stack[--stackSize];
        if (current == NULL_NODE)
            continue;

        // if the given AABB does not overlap the bounds of the current node, continue and thus discard all child nodes
        if (!AABBHasOverlap(&tree->nodes[current].bounds, query_box))
            continue;

        // if the given AABB overlaps current node and it is a leaf, add it to the results
        if (AABBTreeNode_isLeaf(&tree->nodes[current]))
        {
            results[*result_count] = current;
            (*result_count)++;
            // if the maximum of allowed results is reached, break
            if (*result_count >= max_results)
            {
                break;
            }
        }
        else
        {
            if (stackSize + 2 > 256)
            {
                // Handle stack overflow (e.g., log an error, return an error code, etc.)
                // For this example, we'll just return to avoid overflow
                return;
            }
            // if the AABB overlaps the current node and it is not a leaf, add the children to the stack
            stack[stackSize++] = tree->nodes[current].left;
            stack[stackSize++] = tree->nodes[current].right;
        }
    }
}

/// @brief Query the AABBTree for (leaf) nodes that contain a point
/// @param tree BVH tree
/// @param point the point to query for
/// @param results the pre-initialized array of NodeProxies to store the results
/// @param result_count the amount of results found
/// @param max_results the maximum amount of results to find
void AABBTree_queryPoint(AABBTree *tree, struct Vector3 point, NodeProxy *results, int *result_count, int max_results)
{

    // return if the tree is empty
    if (tree->root == NULL_NODE)
    {
        return;
    }

    // initialize the stack with the root node
    NodeProxy stack[256];
    int stackSize = 0;
    stack[stackSize++] = tree->root;
    NodeProxy current;
    while (stackSize > 0)
    {
        NodeProxy current = stack[--stackSize];
        if (current == NULL_NODE)
            continue;

        // if the point is not inside the bounds of the current node, continue and thus discard all child nodes
        if (!AABBContainsPoint(&tree->nodes[current].bounds, &point))
            continue;

        // if the point is inside the current node and it is a leaf, add it to the results
        if (AABBTreeNode_isLeaf(&tree->nodes[current]))
        {
            results[*result_count] = current;
            (*result_count)++;
            // if the maximum of allowed results is reached, break
            if (*result_count >= max_results)
            {
                break;
            }
        }
        else
        {
            if (stackSize + 2 > 256)
            {
                // Handle stack overflow (e.g., log an error, return an error code, etc.)
                // For this example, we'll just return to avoid overflow
                return;
            }
            // if the AABB overlaps the current node and it is not a leaf, add the children to the stack
            stack[stackSize++] = tree->nodes[current].left;
            stack[stackSize++] = tree->nodes[current].right;
        }
    }
}

void AABBTree_queryRay(AABBTree *tree, struct RayCast* ray, NodeProxy *results, int *result_count, int max_results){
    // return if the tree is empty
    if (tree->root == NULL_NODE)
    {
        return;
    }

    // initialize the stack with the root node
    NodeProxy stack[256];
    int stackSize = 0;
    stack[stackSize++] = tree->root;
    NodeProxy current;
    while (stackSize > 0)
    {
        NodeProxy current = stack[--stackSize];
        if (current == NULL_NODE)
            continue;

        // if the ray does not intersect the bounds of the current node, continue and thus discard all child nodes
        if (!AABBIntersectsRay(&tree->nodes[current].bounds, ray))
            continue;

        // if the ray intersects the current node and it is a leaf, add it to the results
        if (AABBTreeNode_isLeaf(&tree->nodes[current]))
        {
            results[*result_count] = current;
            (*result_count)++;
            // if the maximum of allowed results is reached, break
            if (*result_count >= max_results)
            {
                break;
            }
        }
        // if the ray intersects the current node and it is not a leaf, add the children to the stack
        else
        {

            if (stackSize + 2 > 256)
            {
                // Handle stack overflow (e.g., log an error, return an error code, etc.)
                // For this example, we'll just return to avoid overflow
                return;
            }
            // if the AABB overlaps the current node and it is not a leaf, add the children to the stack
            stack[stackSize++] = tree->nodes[current].left;
            stack[stackSize++] = tree->nodes[current].right;
        }
    }
}