#ifndef __MATH_TRANSFORM_SINGLE_AXIS_H__
#define __MATH_TRANSFORM_SINGLE_AXIS_H__

#include "vector3.h"
#include "vector2.h"
#include "matrix.h"

typedef struct TransformSingleAxis {
    Vector3 position;
    Vector2 rotation;
    Vector3 scale;
} TransformSingleAxis;

void transformSAToMatrix(TransformSingleAxis* transform, float mtx[4][4]);

#endif