#ifndef __MATH_TRANSFORM_SINGLE_AXIS_H__
#define __MATH_TRANSFORM_SINGLE_AXIS_H__

#include "vector3.h"
#include "vector2.h"
#include "matrix.h"

struct TransformSingleAxis {
    Vector3 position;
    Vector2 rotation;
};

void transformSAToMatrix(struct TransformSingleAxis* transform, mat4x4 matrix);

#endif