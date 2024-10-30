#ifndef __COLLISION_SHAPE_SWEEP_H__
#define __COLLISION_SHAPE_SWEEP_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

void sweep_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output);
void sweep_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box);

#endif