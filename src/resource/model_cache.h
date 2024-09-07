#ifndef __RESOURCE_MODEL_CACHE_H__
#define __RESOURCE_MODEL_CACHE_H__

#include "../render/model.h"
#include <malloc.h>

struct model* model_cache_load(const char* filename);
void model_cache_release(struct model* model);

#endif