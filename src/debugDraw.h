#pragma once

#include "./collision/aabbtree.h"
#include <t3d/t3d.h>

/**
 * !!!! NOTE: !!!!
 *
 * These functions here are only for debugging purposes and should not be used in production code!
 * Specifically drawing with the CPU is not ideal, but in this specific case more convenient
 *
 * Manually iterating the BVH tree is also not recommended, since the implementation may change in the future,
 * only interact with it via the t3d_* API!
 */

static void debugDrawLine(uint16_t *fb, int px0, int py0, int px1, int py1, uint16_t color)
{
  uint32_t width = display_get_width();
  uint32_t height = display_get_height();
  if ((px0 > width + 200) || (px1 > width + 200) ||
      (py0 > height + 200) || (py1 > height + 200))
  {
    return;
  }

  float pos[2] = {px0, py0};
  int dx = px1 - px0;
  int dy = py1 - py0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
  if(steps <= 0)return;
  float xInc = dx / (float)steps;
  float yInc = dy / (float)steps;

  for(int i=0; i<steps; ++i)
  {
    if(pos[1] >= 0 && pos[1] < height && pos[0] >= 0 && pos[0] < width) {
      fb[(int)pos[1] * width + (int)pos[0]] = color;
    }
    pos[0] += xInc;
    pos[1] += yInc;
  }
}

static inline void debugDrawLineVec3(uint16_t *fb, const T3DVec3 *p0, const T3DVec3 *p1, uint16_t color)
{
  debugDrawLine(fb, (int)p0->v[0], (int)p0->v[1], (int)p1->v[0], (int)p1->v[1], color);
}

static void debugDrawAABB(uint16_t *fb, Vector3 *min, const Vector3 *max, T3DViewport *vp, float scale, uint16_t color)
{
  // transform min/max to screen space
  T3DVec3 points[8];
  T3DVec3 pt0 = {{min->x*scale, min->y*scale, min->z*scale}};
  T3DVec3 pt1 = {{max->x*scale, min->y*scale, min->z*scale}};

  t3d_viewport_calc_viewspace_pos(vp, &points[0], &pt0);
  t3d_viewport_calc_viewspace_pos(vp, &points[1], &pt1); pt0.v[1] = max->y*scale;
  t3d_viewport_calc_viewspace_pos(vp, &points[2], &pt0); pt1.v[1] = max->y*scale;
  t3d_viewport_calc_viewspace_pos(vp, &points[3], &pt1); pt0.v[2] = max->z*scale;
  t3d_viewport_calc_viewspace_pos(vp, &points[4], &pt0); pt1.v[2] = max->z*scale;
  t3d_viewport_calc_viewspace_pos(vp, &points[5], &pt1); pt0.v[1] = min->y*scale;
  t3d_viewport_calc_viewspace_pos(vp, &points[6], &pt0); pt1.v[1] = min->y*scale;
  t3d_viewport_calc_viewspace_pos(vp, &points[7], &pt1);

  // draw min/max as wireframe cube
  const int indices[24] = {
    0, 1, 1, 3, 3, 2, 2, 0,
    4, 5, 5, 7, 7, 6, 6, 4,
    0, 6, 1, 7, 2, 4, 3, 5
  };
  for(int i=0; i<24; i+=2) {
    debugDrawLineVec3(fb, &points[indices[i]], &points[indices[i+1]], color);
  }
}

static uint16_t DEBUG_COLORS[8] = {
  0x037f, 0x92ff, 0xca7f, 0xeab9,
  0xfb31, 0xfbe7, 0xfc9b, 0xfd41,
};

static void debugDrawBVTreeNode(
  uint16_t *fb, T3DViewport *vp,
  NodeProxy node, AABBTree *tree, const T3DFrustum *frustum, float scale, int level, int max_level
) {

  if(node != AABBTREE_NULL_NODE && level <= max_level) {
    
    if(t3d_frustum_vs_aabb(frustum, (T3DVec3*)(&tree->nodes[node].bounds.min), (T3DVec3*)(&tree->nodes[node].bounds.max))) {
      if(AABBTreeNode_isLeaf(&tree->nodes[node]))debugDrawAABB(fb, &tree->nodes[node].bounds.min, &tree->nodes[node].bounds.max, vp, scale, DEBUG_COLORS[level & 7]);
      debugDrawBVTreeNode(fb, vp, tree->nodes[node]._left, tree, frustum, scale, level+1, max_level);
      debugDrawBVTreeNode(fb, vp, tree->nodes[node]._right, tree, frustum, scale, level+1, max_level);
    }
  }

  
}

static void debugDrawBVTree(uint16_t *fb, AABBTree *tree, T3DViewport *vp, const T3DFrustum *frustum, float scale, int start_level, int max_level)
{
  debugDrawBVTreeNode(fb, vp, tree->root, tree, frustum, scale, start_level, max_level);


}