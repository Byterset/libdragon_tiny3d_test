#ifndef __RENDER_RENDER_BATCH_H__
#define __RENDER_RENDER_BATCH_H__

#include <libdragon.h>
#include <t3d/t3d.h>

#include <t3d/t3dmodel.h>
#include <t3d/t3dskeleton.h>
#include "frame_alloc.h"

#include "../math/matrix.h"
#include "../math/transform.h"

#define RENDER_BATCH_MAX_SIZE   256
#define RENDER_BATCH_TRANSFORM_COUNT    64
#define MAX_BILLBOARD_SPRITES           128

struct render_billboard_sprite {
    struct Vector3 position;
    float radius;
    color_t color;
};

enum render_batch_type {
    RENDER_BATCH_MODEL,
    RENDER_BATCH_BILLBOARD,
    RENDER_BATCH_CALLBACK,
};

struct render_batch_billboard_element {
    struct render_billboard_sprite* sprites;
    uint16_t sprite_count;
};

struct render_batch;

typedef void (*RenderCallback)(void* data, struct render_batch* batch);

struct render_batch_element {
    T3DMaterial* material;
    uint16_t type;
    union {
        struct {
            rspq_block_t* block;
            T3DMat4FP* transform;
            T3DSkeleton* skeleton;
            T3DMat4FP* tmp_fixed_pose;
        } model;
        struct render_batch_billboard_element billboard;
        struct {
            RenderCallback callback;
            void* data;
        } callback;
    };
};

struct render_batch {
    mat4x4 camera_matrix;
    struct frame_memory_pool* pool;
    struct render_batch_element elements[RENDER_BATCH_MAX_SIZE];
    short element_count;
};

void render_batch_init(struct render_batch* batch, struct Transform* camera_transform, struct frame_memory_pool* pool);

struct render_batch_element* render_batch_add(struct render_batch* batch);

void render_batch_add_t3dmodel(struct render_batch* batch, rspq_block_t* block, T3DMat4FP* transform, T3DSkeleton* skeleton);

void render_batch_add_callback(struct render_batch* batch, T3DMaterial* material, RenderCallback callback, void* data);
// caller is responsible for populating sprite list
// the sprite count returned may be less than the sprite count requested
struct render_batch_billboard_element* render_batch_add_particles(struct render_batch* batch, T3DMaterial* material, int count);

struct render_batch_billboard_element render_batch_get_sprites(struct render_batch* batch, int count);
mat4x4* render_batch_get_transform(struct render_batch* batch);
T3DMat4FP* render_batch_get_transformfp(struct render_batch* batch);

void render_batch_finish(struct render_batch* batch, mat4x4 view_proj, T3DViewport* viewport);

#endif