#ifndef __COLLISION_SHAPE_LINESEGMENT_H__
#define __COLLISION_SHAPE_LINESEGMENT_H__

#include "../../math/vector2.h"
#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../../math/aabb.h"

typedef struct { Vector3 segment_start; Vector3 segment_end;} line_segment;

void lineSegment_support_function(void* data, Vector3* direction, Vector3* output);
void lineSegment_bounding_box(void* data, Quaternion* rotation, AABB* box);

#endif