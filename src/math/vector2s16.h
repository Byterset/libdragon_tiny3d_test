#ifndef __MATH_VECTOR2S16_H__
#define __MATH_VECTOR2S16_H__

#include "vector3.h"

typedef struct Vector2s16 {
    union {
        struct {
            short x;
            short y;
        };
        int equalTest;
    };
} Vector2s16;

#define VECTOR2s16_AS_ARRAY(vector) ((s16*)(vector))

void vector2s16Add(Vector2s16* a, Vector2s16* b, Vector2s16* output);
void vector2s16Sub(Vector2s16* a, Vector2s16* b, Vector2s16* output);

int vector2s16Dot(Vector2s16* a, Vector2s16* b);
int vector2s16Cross(Vector2s16* a, Vector2s16* b);
int vector2s16MagSqr(Vector2s16* a);
int vector2s16DistSqr(Vector2s16* a, Vector2s16* b);

int vector2s16FallsBetween(Vector2s16* from, Vector2s16* towards, Vector2s16* check);

void vector2s16Barycentric(Vector2s16* a, Vector2s16* b, Vector2s16* c, Vector2s16* point, Vector3* output);

#endif