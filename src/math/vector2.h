#ifndef _MATH_VECTOR2_H
#define _MATH_VECTOR2_H

typedef union Vector2 {
    struct {float x, y;};
    float data[2];
} Vector2;

extern Vector2 gRight2;
extern Vector2 gUp2;
extern Vector2 gZeroVec2;
extern Vector2 gOneVec2;

typedef union Vector3 Vector3;

void vector2ComplexMul(Vector2* a, Vector2* b, Vector2* out);
void vector2ComplexConj(Vector2* a, Vector2* out);
int vector2RotateTowards(Vector2* from, Vector2* towards, Vector2* max, Vector2* out);
void vector2ComplexFromAngleRad(float radians, Vector2* out);
void vector2ComplexFromAngleDeg(float degrees, Vector2* out);
void vector2Rotate90(Vector2* input, Vector2* out);
float vector2Cross(Vector2* a, Vector2* b);
float vector2Dot(Vector2* a, Vector2* b);
float vector2MagSqr(Vector2* a);
float vector2DistSqr(Vector2* a, Vector2* b);
void vector2Add(Vector2* a, Vector2* b, Vector2* out);
void vector2Scale(Vector2* a, float scale, Vector2* out);
void vector2Normalize(Vector2* a, Vector2* out);
void vector2Sub(Vector2* a, Vector2* b, Vector2* out);
void vector2Negate(Vector2* a, Vector2* out);

void vector2Min(Vector2* a, Vector2* b, Vector2* out);
void vector2Max(Vector2* a, Vector2* b, Vector2* out);

void vector2Lerp(Vector2* a, Vector2* b, float lerp, Vector2* out);

void vector2RandomUnitCircle(Vector2* result);

void vector2LookDir(Vector2* result, Vector3* direction);
void vector3RotatedSpeed(Vector2* rotation, Vector3* result, float speed);

#endif