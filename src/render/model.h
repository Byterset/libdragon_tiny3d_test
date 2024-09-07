#ifndef __RENDER_MODEL_H__
#define __RENDER_MODEL_H__

#include <libdragon.h>
#include <t3d/t3dmodel.h>
#include <t3d/t3dskeleton.h>
#include <stdbool.h>


struct model {
    rspq_block_t* block;
    T3DModel* t3d_model;
    T3DSkeleton skeleton;
    bool has_skeleton;
};

// used to directly load a model
// models loaded this way must be 
// released with model_release
void model_load(struct model* into, const char* model_filename);
void model_release(struct model* model);

#endif