#ifndef __RENDER_DEFS_H__
#define __RENDER_DEFS_H__

#include "../math/vector3.h"

// This is the scale of the scene, all positions, scales etc get scaled by this value internally, it should be the same as the model export scale in MAKEFILE
#define SCENE_SCALE 16.0f

#define DEBUG_COLLIDERS_RAYLIB = 1

void pack_position_vector(struct Vector3* input, short output[3]);

#endif