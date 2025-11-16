#ifndef __COLLISION_SHAPE_SWEEP_H__
#define __COLLISION_SHAPE_SWEEP_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

void sweep_support_function(const void* data, const Vector3* direction, Vector3* output);
void sweep_bounding_box(const void* data, const Quaternion* rotation, AABB* box);

#endif