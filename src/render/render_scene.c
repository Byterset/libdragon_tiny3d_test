#include "render_scene.h"

#include "../util/blist.h"
#include <malloc.h>
#include <stdbool.h>
#include "defs.h"

#define MIN_RENDER_SCENE_SIZE   64

struct render_scene r_scene_3d;

void render_scene_reset() {
    callback_list_reset(&r_scene_3d.callbacks, sizeof(struct render_scene_element), MIN_RENDER_SCENE_SIZE, NULL);
}

/// @brief Add a callback to the render scene that will be executed on every render
/// @param center the center of the bounding sphere, if NULL the element will not be culled
/// @param radius the radius of the bounding sphere (should be large enough to fit the object at any rotation)
/// @param callback the render callback to execute 
/// @param data data to pass to the callback
void render_scene_add_callback(Vector3* center, float radius, render_scene_callback callback, void* data) {
    struct render_scene_element element;

    element.data = data;
    element.center = center;
    element.radius = radius;

    callback_list_insert_with_id(&r_scene_3d.callbacks, callback, &element, (callback_id)data);
}

/// @brief premade callback for adding a renderable consisting of a transform and a t3d model to a batch that will then be rendered in bulk
/// @param data the renderable to render
/// @param batch the batch to add the renderable to
void render_scene_render_renderable(void* data, struct render_batch* batch) {
    struct renderable* renderable = (struct renderable*)data;

    T3DMat4FP* mtxfp = render_batch_get_transformfp(batch);

    if (!mtxfp) {
        return;
    }

    Matrix4x4 mtx;
    transformToMatrix(renderable->transform, mtx.m);

    t3d_mat4_to_fixed_3x4(mtxfp, (T3DMat4*)mtx.m);

    render_batch_add_t3dmodel(batch, renderable->model, mtxfp);
}

/// @brief premade callback for adding a single axis renderable consisting of a transform and a t3d model to a batch that will then be rendered in bulk
/// @param data the single axis renderable to render
/// @param batch the batch to add the renderable to
void render_scene_render_renderable_single_axis(void* data, struct render_batch* batch) {
    struct renderable_single_axis* renderable = (struct renderable_single_axis*)data;

    T3DMat4FP* mtxfp = render_batch_get_transformfp(batch);

    if (!mtxfp) {
        return;
    }

    Matrix4x4 mtx;
    transformSAToMatrix(renderable->transform, mtx.m);

    t3d_mat4_to_fixed_3x4(mtxfp, (T3DMat4*)mtx.m);

    render_batch_add_t3dmodel(batch, renderable->model, mtxfp);
}

/// @brief Add the render_renderable callback to the list of callbacks with the renderable position as the culling center
/// and the renderable as data
/// @param renderable 
/// @param radius 
void render_scene_add_renderable(struct renderable* renderable, float radius) {
    render_scene_add_callback(&renderable->transform->position, radius, render_scene_render_renderable, renderable);
}

/// @brief Add the render_renderable_single_axis callback to the list of callbacks with the renderable position as the culling center
/// and the renderable as data
/// @param renderable 
/// @param radius 
void render_scene_add_renderable_single_axis(struct renderable_single_axis* renderable, float radius) {
    render_scene_add_callback(&renderable->transform->position, radius, render_scene_render_renderable_single_axis, renderable);
}

/// @brief remove a callback from the render scene
/// @param data the pointer to the data that was passed with the callback when adding it
void render_scene_remove(void* data) {
    callback_list_remove(&r_scene_3d.callbacks, (callback_id)data);
}


/// @brief Render the scene
///
/// This will apply viewport settings, cull elements outside the frustum and execute the render callbacks.
/// If the callbacks added elements to the batch, they will be rendered in bulk.
/// @param camera 
/// @param viewport 
/// @param pool 
/// @param fog 
void render_scene_render(struct camera* camera, T3DViewport* viewport, struct frame_memory_pool* pool, struct render_fog_params* fog) {
    struct render_batch batch;

    render_batch_init(&batch, &camera->transform, pool);

    struct callback_element* current = callback_list_get(&r_scene_3d.callbacks, 0);

    int culled = 0;

    // Run all the callbacks that will add something to the batch or render directly
    for (int i = 0; i < r_scene_3d.callbacks.count; ++i) {
        struct render_scene_element* el = callback_element_get_data(current);

        // Skip elements outside the frustum, only check if center of Boundingsphere is set
        if(el->center) {
            if (!t3d_frustum_vs_sphere(&viewport->viewFrustum, (T3DVec3*)el->center, el->radius))
            {
                current = callback_list_next(&r_scene_3d.callbacks, current);
                culled++;
                continue;
            }
        }

        ((render_scene_callback)current->callback)(el->data, &batch);
        current = callback_list_next(&r_scene_3d.callbacks, current);
    }

    // Execute drawing of batch elements
    render_batch_execute(&batch, viewport->matCamProj, viewport, fog);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, 16, 104, "culled: %d", culled);
}