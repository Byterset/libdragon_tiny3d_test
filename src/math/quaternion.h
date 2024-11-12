
#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "vector3.h"
#include "vector2.h"

typedef struct Quaternion {
    float x, y, z, w;
} Quaternion;

extern Quaternion gQuaternionZero;

void quatIdent(Quaternion* q);
void quatAxisAngle(struct Vector3* axis, float angle, Quaternion* out);
void quatAxisComplex(struct Vector3* axis, struct Vector2* complex, Quaternion* out);
void quatConjugate(Quaternion* in, Quaternion* out);
void quatNegate(Quaternion* in, Quaternion* out);
void quatMultVector(Quaternion* q, struct Vector3* a, struct Vector3* out);
void quatRotatedBoundingBoxSize(Quaternion* q, struct Vector3* halfBoxSize, struct Vector3* out);
void quatMultiply(Quaternion* a, Quaternion* b, Quaternion* out);
void quatAdd(Quaternion* a, Quaternion* b, Quaternion* out);
void quatToMatrix(Quaternion* q, float out[4][4]);
void quatNormalize(Quaternion* q, Quaternion* out);
void quatRandom(Quaternion* q);
void quatLook(struct Vector3* lookDir, struct Vector3* up, Quaternion* out);
void quatEulerAngles(struct Vector3* angles, Quaternion* out);
// cheap approximation of slerp
void quatLerp(Quaternion* a, Quaternion* b, float t, Quaternion* out);
void quatApplyAngularVelocity(Quaternion* input, struct Vector3* w, float timeStep, Quaternion* output);
void quatDecompose(Quaternion* input, struct Vector3* axis, float* angle);
void quatRotateAxisEuler(Quaternion* q, struct Vector3* axis, float angle, Quaternion* out);
int quatIsIdentical(Quaternion* a, Quaternion* b);

float quatDot(Quaternion* a, Quaternion* b);

#endif