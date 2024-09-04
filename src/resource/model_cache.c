#include "model_cache.h"

#include "resource_cache.h"

struct resource_cache model_resource_cache;

/// @brief Will look for the model in the associated resource cache. If the model is not in the cache, it will be loaded and added to the cache.
/// @param filename path to the model in the rom file system
/// @return 
T3DModel* model_cache_load(const char* filename) {
    struct resource_cache_entry* entry = resource_cache_use(&model_resource_cache, filename);

    if (!entry->resource) {

        T3DModel* result = t3d_model_load(filename);

        entry->resource = result;
    }

    return entry->resource;
}

/// @brief Attempts to remove model from the associated resource cache. If the model is not in the cache or still referenced, it will not be freed.
/// @param model pointer to the model to be freed
void model_cache_release(T3DModel* model) {
    if (resource_cache_free(&model_resource_cache, model)) {
        t3d_model_free(model);
    }
}