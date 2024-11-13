
#ifndef _VECTOR4_H
#define _VECTOR4_H

typedef struct Vector4 {
    float x, y, z, w;
} Vector4;

void vector4Lerp(Vector4* a, Vector4* b, float lerp, Vector4* out);

#endif