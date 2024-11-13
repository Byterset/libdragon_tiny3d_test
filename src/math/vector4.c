#include "vector4.h"

void vector4Lerp(Vector4* a, Vector4* b, float lerp, Vector4* out) {
    float tInv = 1.0f - lerp;

    out->x = a->x * tInv + b->x * lerp;
    out->y = a->y * tInv + b->y * lerp;
    out->z = a->z * tInv + b->z * lerp;
    out->w = a->w * tInv + b->w * lerp;
}