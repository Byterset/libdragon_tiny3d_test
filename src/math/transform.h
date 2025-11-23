#ifndef _TRANSFORM_H
#define _TRANSFORM_H

#include "vector3.h"
#include "quaternion.h"
#include "matrix.h"

typedef struct Transform {
    Vector3 position;
    Quaternion rotation;
    Vector3 scale;
} Transform;

void transformInitIdentity(Transform* in);
void transformToMatrix(Transform* in, Matrix4x4* mtx);
void transformInvert(Transform* in, Transform* out);
void transformPoint(Transform* transform, Vector3* in, Vector3* out);
void transformPointInverse(Transform* transform, Vector3* in, Vector3* out);
void transformPointInverseNoScale(Transform* transform, Vector3* in, Vector3* out);
void transformConcat(Transform* left, Transform* right, Transform* output);

void transformLerp(Transform* a, Transform* b, float t, Transform* output);

#endif