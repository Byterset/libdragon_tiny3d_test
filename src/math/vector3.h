
#ifndef _VECTOR3_H
#define _VECTOR3_H

#include <libdragon.h>

typedef fm_vec3_t Vector3;

typedef union Vector3u8 {
    struct {char x, y, z;};
    char v[3];
} Vector3u8;

extern Vector3 gRight;
extern Vector3 gUp;
extern Vector3 gDown;
extern Vector3 gForward;
extern Vector3 gZeroVec;
extern Vector3 gOneVec;

void vector3Copy(Vector3* source, Vector3* target);
void vector3Abs(Vector3* in, Vector3* out);
void vector3Negate(Vector3* in, Vector3* out);
void vector3Scale(Vector3* in, Vector3* out, float scale);
void vector3Add(Vector3* a, Vector3* b, Vector3* out);
void vector3AddToSelf(Vector3* a, Vector3* b);
void vector3AddScaled(Vector3* a, Vector3* normal, float scale, Vector3* out);
void vector3Sub(Vector3* a, Vector3* b, Vector3* out);
void vector3SubFromSelf(Vector3* a, Vector3* b);
void vector3Multiply(Vector3* a, Vector3* b, Vector3* out);
void vector3Normalize(Vector3* in, Vector3* out);
void vector3NormalizeSelf(Vector3* self);
void vector3Lerp(Vector3* a, Vector3* b, float t, Vector3* out);
float vector3Dot(Vector3* a, Vector3* b);
float vector3MagSqrd(Vector3* a);
float vector3Mag(Vector3* a);
float vector3DistSqrd(Vector3* a, Vector3* b);
float vector3Dist(Vector3* a, Vector3* b);
void vector3Cross(Vector3* a, Vector3* b, Vector3* out);
void vector3Perp(Vector3* a, Vector3* out);
void vector3Project(Vector3* in, Vector3* normal, Vector3* out);
void vector3ProjectPlane(Vector3* in, Vector3* normal, Vector3* out);
int vector3MoveTowards(Vector3* from, Vector3* towards, float maxDistance, Vector3* out);
void vector3TripleProduct(Vector3* a, Vector3* b, Vector3* c, Vector3* output);

void vector3Max(Vector3* a, Vector3* b, Vector3* out);
void vector3Min(Vector3* a, Vector3* b, Vector3* out);

int vector3IsZero(Vector3* vector);
int vector3Equals(Vector3* a, Vector3* b);

void vector3ToVector3u8(Vector3* input, Vector3u8* output);

float vector3EvalBarycentric1D(Vector3* baryCoords, float a, float b, float c);

#endif