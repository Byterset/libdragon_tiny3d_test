#include "model.h"

#include <t3d/t3d.h>
#include "../resource/sprite_cache.h"

/// @brief Load a T3D model from a file and store it in the model struct. Also load the skeleton if the model has one.
/// Depending on if a Skeleton exists, the rspq_block will be generated to draw the model skinned or unskinned.
/// The generated RSPQ Block is a default t3d_model_draw block, to customize the drawing, create a callback to the custom draw function and add it to the renderbatch.
/// @param into the model struct to store the loaded model
/// @param model_filename the path to the T3DModel in the rom file system
void model_load(struct model* into, const char* model_filename) {
    into->t3d_model = t3d_model_load(model_filename);

    into->has_skeleton = t3d_model_get_skeleton(into->t3d_model) != NULL;

    if(into->has_skeleton){
        into->skeleton = t3d_skeleton_create(into->t3d_model);
    }

    // generate the block to draw the model skinned or unskinned
    // this is a default t3d_model_draw block, to customize the drawing, create a callback to the custom draw function and add it to the renderbatch
    rspq_block_begin();
        if(into->has_skeleton){
            t3d_model_draw_skinned(into->t3d_model, &into->skeleton);
        }
        else{
            t3d_model_draw(into->t3d_model);
        }
    into->t3d_model->userBlock = rspq_block_end();
}

/// @brief Free the memory associated with the model struct. 
/// This will also destroy the associated T3DModel & skeleton if it exists as well as free the rspq_block.
/// @param model 
void model_release(struct model* model) {
    t3d_model_free(model->t3d_model);
    model->t3d_model = NULL;
    if(model->has_skeleton){
        t3d_skeleton_destroy(&model->skeleton);
        model->has_skeleton = false;
    }
    free(model);
}