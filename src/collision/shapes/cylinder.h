#ifndef __COLLISION_SHAPE_CYLINDER_H__
#define __COLLISION_SHAPE_CYLINDER_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

void cylinder_minkowski_sum(void* data, struct Vector3* direction, struct Vector3* output);
void cylinder_bounding_box(void* data, struct Quaternion* rotation, struct AABB* box);

#endif