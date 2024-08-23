#ifndef __RESOURCE_MODEL_CACHE_H__
#define __RESOURCE_MODEL_CACHE_H__

#include <t3d/t3d.h>
#include <t3d/t3dmath.h>
#include <t3d/t3dmodel.h>

T3DModel* model_cache_load(const char* filename);
void model_cache_release(T3DModel* model);

#endif