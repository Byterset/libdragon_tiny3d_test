
#ifndef _VECTOR4_H
#define _VECTOR4_H

#include <libdragon.h>

typedef fm_vec4_t Vector4;

void vector4Lerp(Vector4* a, Vector4* b, float lerp, Vector4* out);

#endif