#ifndef __COLLISION_SHAPE_BOX_H__
#define __COLLISION_SHAPE_BOX_H__

#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../../math/aabb.h"

void box_support_function(const void* data, const Vector3* direction, Vector3* output);
void box_bounding_box(const void* data, const Quaternion* rotation, AABB* box);
void box_inertia_tensor(void* data, float mass, Vector3* out);

#endif