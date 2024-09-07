#include "renderable.h"

/// @brief initializes a renderable object with a given transform and T3D model path
/// @param renderable pointer to the renderable
/// @param transform pointer to the transform
/// @param model_filename path string to the model in the rom file system
void renderable_init(struct renderable* renderable, struct Transform* transform, const char* model_filename) {
    renderable->transform = transform;
    renderable->model = model_cache_load(model_filename);
}

/// @brief free the memory of a renderable object.
/// This will also destroy the associated model, T3DModel & skeleton as well as remove it from the cache
/// @param renderable 
void renderable_destroy(struct renderable* renderable) {
    model_cache_release(renderable->model);
    renderable->model = NULL;
}

void renderable_single_axis_init(struct renderable_single_axis* renderable, struct TransformSingleAxis* transform, const char* model_filename) {
    renderable->transform = transform;
    renderable->model = model_cache_load(model_filename);
}

/// @brief free the memory of a renderable single axis object.
/// This will also destroy the associated model, T3DModel & skeleton as well as remove it from the cache
/// @param renderable 
void renderable_single_axis_destroy(struct renderable_single_axis* renderable) {
    model_cache_release(renderable->model);
    renderable->model = NULL;
}