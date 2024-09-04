#include "renderable.h"

/// @brief initializes a renderable object with a given transform and T3D model
/// @param renderable pointer to the renderable
/// @param transform pointer to the transform
/// @param model_filename path string to the model in the rom file system
void renderable_init(struct renderable* renderable, struct Transform* transform, const char* model_filename) {
    renderable->transform = transform;
    renderable->model = model_cache_load(model_filename);
}

/// @brief free the memory of a renderable object
/// @param renderable 
void renderable_destroy(struct renderable* renderable) {
    if(renderable->skeleton.skeletonRef != NULL){
        t3d_skeleton_destroy(&renderable->skeleton);
    }
    model_cache_release(renderable->model);
    renderable->model = NULL;
    rspq_block_free(renderable->block);
}

void renderable_single_axis_init(struct renderable_single_axis* renderable, struct TransformSingleAxis* transform, const char* model_filename) {
    renderable->transform = transform;
    renderable->model = model_cache_load(model_filename);
    if(t3d_model_get_skeleton(renderable->model) != NULL){
        renderable->skeleton = t3d_skeleton_create(renderable->model);
    }
}

void renderable_single_axis_destroy(struct renderable_single_axis* renderable) {
    t3d_skeleton_destroy(&renderable->skeleton);
    model_cache_release(renderable->model);
    renderable->model = NULL;
}