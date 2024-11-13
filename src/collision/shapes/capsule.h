#ifndef __COLLISION_SHAPE_CAPSULE_H__
#define __COLLISION_SHAPE_CAPSULE_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

void capsule_support_function(void* data, Vector3* direction, Vector3* output);
void capsule_bounding_box(void* data, Quaternion* rotation, AABB* box);

#endif