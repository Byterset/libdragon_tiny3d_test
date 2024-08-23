#include "model_cache.h"

#include "resource_cache.h"

struct resource_cache model_resource_cache;

T3DModel* model_cache_load(const char* filename) {
    struct resource_cache_entry* entry = resource_cache_use(&model_resource_cache, filename);

    if (!entry->resource) {

        T3DModel* result = t3d_model_load(filename);

        entry->resource = result;
    }

    return entry->resource;
}

void model_cache_release(T3DModel* model) {
    if (resource_cache_free(&model_resource_cache, model)) {
        t3d_model_free(model);
    }
}