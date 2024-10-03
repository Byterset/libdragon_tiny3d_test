#ifndef __RENDER_RENDER_BATCH_H__
#define __RENDER_RENDER_BATCH_H__

#include <libdragon.h>
#include <t3d/t3d.h>
#include <t3d/t3dmodel.h>
#include <t3d/t3dskeleton.h>

#include "material.h"
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

struct render_fog_params {
    bool enabled;
    float start;
    float end;
    color_t color;
};

enum render_batch_type {
    RENDER_BATCH_SKYBOX, // Skybox consisting of a sub-texture that is blit to the framebuffer
    RENDER_BATCH_MODEL, // Tiny3D model
    RENDER_BATCH_BILLBOARD, // A collection of Sprites with a custom material block
    RENDER_BATCH_CALLBACK, // A custom render callback that will be executed as is by the renderer
    RENDER_BATCH_EQUIDISTANT, // A physical model that is always rendered at the same position relative to the camera
    
};

struct render_batch_billboard_element {
    struct render_billboard_sprite* sprites;
    uint16_t sprite_count;
};

struct render_batch;

typedef void (*RenderCallback)(void* data, struct render_batch* batch);

struct render_batch_element {
    struct material* material;
    uint16_t type;
    union {
        struct {
            surface_t* surface;
        } skybox;
        struct {
            rspq_block_t* block;
            T3DMat4FP* transform;
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

void render_batch_add_t3dmodel(struct render_batch* batch, rspq_block_t* block, T3DMat4FP* transform, T3DSkeleton* skeleton);

void render_batch_add_callback(struct render_batch* batch, struct material* material, RenderCallback callback, void* data);
// caller is responsible for populating sprite list
// the sprite count returned may be less than the sprite count requested
struct render_batch_billboard_element* render_batch_add_particles(struct render_batch* batch, struct material* material, int count);

void render_batch_add_equidistant(struct render_batch* batch, rspq_block_t* block);

void render_batch_add_skybox_flat(struct render_batch* batch, surface_t* surface);

struct render_batch_billboard_element render_batch_get_sprites(struct render_batch* batch, int count);
mat4x4* render_batch_get_transform(struct render_batch* batch);
T3DMat4FP* render_batch_get_transformfp(struct render_batch* batch);

void render_batch_execute(struct render_batch* batch, mat4x4 view_proj, T3DViewport* viewport, struct render_fog_params* fog);

#endif