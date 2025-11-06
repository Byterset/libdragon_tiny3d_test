#ifndef __COLLISION_SHAPE_SPHERE_H__
#define __COLLISION_SHAPE_SPHERE_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/aabb.h"
#include "../../math/quaternion.h"

void sphere_support_function(void* data, Vector3* direction, Vector3* output);
void sphere_bounding_box(void* data, Quaternion* rotation, AABB* box);
void sphere_inertia_tensor(void* data, float mass, Vector3* out);

#endif