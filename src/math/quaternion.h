
#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "vector3.h"
#include "vector2.h"

typedef struct Quaternion {
    float x, y, z, w;
} Quaternion;

extern Quaternion gQuaternionZero;

void quatIdent(Quaternion* q);
void quatAxisAngle(Vector3* axis, float angle, Quaternion* out);
void quatAxisComplex(Vector3* axis, Vector2* complex, Quaternion* out);
void quatConjugate(Quaternion* in, Quaternion* out);
void quatNegate(Quaternion* in, Quaternion* out);
void quatMultVector(Quaternion* q, Vector3* a, Vector3* out);
void quatRotatedBoundingBoxSize(Quaternion* q, Vector3* halfBoxSize, Vector3* out);
void quatMultiply(Quaternion* a, Quaternion* b, Quaternion* out);
void quatAdd(Quaternion* a, Quaternion* b, Quaternion* out);
void quatToMatrix(Quaternion* q, float out[4][4]);
void quatNormalize(Quaternion* q, Quaternion* out);
void quatRandom(Quaternion* q);
void quatLook(Vector3* lookDir, Vector3* up, Quaternion* out);
void quatEulerAngles(Vector3* angles, Quaternion* out);
// cheap approximation of slerp
void quatLerp(Quaternion* a, Quaternion* b, float t, Quaternion* out);
void quatApplyAngularVelocity(Quaternion* input, Vector3* w, float timeStep, Quaternion* output);
void quatDecompose(Quaternion* input, Vector3* axis, float* angle);
void quatRotateAxisEuler(Quaternion* q, Vector3* axis, float angle, Quaternion* out);
int quatIsIdentical(Quaternion* a, Quaternion* b);

float quatDot(Quaternion* a, Quaternion* b);

#endif