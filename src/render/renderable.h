#ifndef __RENDER_RENDERABLE_H__
#define __RENDER_RENDERABLE_H__

#include "../math/transform.h"
#include "../math/transform_single_axis.h"
#include <t3d/t3dmodel.h>
#include <t3d/t3dskeleton.h>
#include "../resource/model_cache.h"
#include "../render/model.h"

struct renderable {
    Transform* transform; //the transform of the object
    struct model* model; //the model of the object
};

void renderable_init(struct renderable* renderable, Transform* transform, const char* model_filename);
void renderable_destroy(struct renderable* renderable);

struct renderable_single_axis {
    TransformSingleAxis* transform;
    struct model* model;
};

void renderable_single_axis_init(struct renderable_single_axis* renderable, TransformSingleAxis* transform, const char* model_filename);
void renderable_single_axis_destroy(struct renderable_single_axis* renderable);

#endif