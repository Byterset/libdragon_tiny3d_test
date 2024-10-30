#ifndef __COLLISION_SHAPE_BOX_H__
#define __COLLISION_SHAPE_BOX_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../../math/aabb.h"

void box_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output);
void box_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box);

#endif