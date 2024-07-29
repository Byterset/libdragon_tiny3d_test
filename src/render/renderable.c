#include "renderable.h"

void renderable_init(struct renderable* renderable, struct Transform* transform, const char* model_filename) {
    renderable->transform = transform;
    renderable->model = t3d_model_load(model_filename);
    if(t3d_model_get_skeleton(renderable->model) != NULL){
        renderable->skeleton = t3d_skeleton_create(renderable->model);
        rspq_block_begin();
        t3d_model_draw_skinned(renderable->model, &renderable->skeleton);
        renderable->block = rspq_block_end();
    }
    else{
        rspq_block_begin();
        t3d_model_draw(renderable->model);
        renderable->block = rspq_block_end();
    }
}

void renderable_destroy(struct renderable* renderable) {
    t3d_skeleton_destroy(&renderable->skeleton);
    t3d_model_free(renderable->model);
    renderable->model = NULL;
    rspq_block_free(renderable->block);
}

void renderable_single_axis_init(struct renderable_single_axis* renderable, struct TransformSingleAxis* transform, const char* model_filename) {
    renderable->transform = transform;
    renderable->model = t3d_model_load(model_filename);
    if(t3d_model_get_skeleton(renderable->model) != NULL){
        renderable->skeleton = t3d_skeleton_create(renderable->model);
    }
}

void renderable_single_axis_destroy(struct renderable_single_axis* renderable) {
    t3d_skeleton_destroy(&renderable->skeleton);
    t3d_model_free(renderable->model);
    renderable->model = NULL;
}