#ifndef __RENDER_DEFS_H__
#define __RENDER_DEFS_H__

// Define the scale that the models have been exported with - see Makefile for more info
#ifndef MODEL_SCALE
#define MODEL_SCALE 1.0f
#endif
#define INV_MODEL_SCALE (1.0f / MODEL_SCALE)
#define FRAMEBUFFER_COUNT 3 // The number of Framebuffers for the game, should be at least 3

// #define DEBUG_COLLIDERS_RAYLIB = 1

#endif