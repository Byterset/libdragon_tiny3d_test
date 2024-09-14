#include "render_batch.h"

#include "../util/sort.h"
#include "../time/time.h"
#include "material.h"
#include "defs.h"

T3DVertPacked billboard_vertices[2];

void render_batch_init(struct render_batch *batch, struct Transform *camera_transform, struct frame_memory_pool *pool)
{
    batch->element_count = 0;
    batch->pool = pool;

    transformToMatrix(camera_transform, batch->camera_matrix);
}

static struct render_batch_element *render_batch_add_init(struct render_batch *batch)
{
    if (batch->element_count >= RENDER_BATCH_MAX_SIZE)
    {
        return NULL;
    }

    struct render_batch_element *result = &batch->elements[batch->element_count];
    ++batch->element_count;

    result->material = NULL;
    result->type = RENDER_BATCH_MODEL;
    result->model.block = 0;
    result->model.transform = NULL;

    return result;
}

void render_batch_add_t3dmodel(struct render_batch *batch, rspq_block_t *block, T3DMat4FP *transform, T3DSkeleton *skeleton)
{
    struct render_batch_element *element = render_batch_add_init(batch);

    if (!element)
    {
        return;
    }

    element->model.block = block;
    element->material = NULL; // T3DModels have their own materials
    element->model.transform = transform;
}

void render_batch_add_callback(struct render_batch *batch, struct material *material, RenderCallback callback, void *data)
{
    struct render_batch_element *element = render_batch_add_init(batch);

    if (!element)
    {
        return;
    }

    element->type = RENDER_BATCH_CALLBACK;
    element->material = material;
    element->callback.callback = callback;
    element->callback.data = data;
}

struct render_batch_billboard_element *render_batch_add_particles(struct render_batch *batch, struct material *material, int count, T3DMat4FP* sprite_mtx)
{
    struct render_batch_element *result = render_batch_add_init(batch);

    result->type = RENDER_BATCH_BILLBOARD;
    result->material = material;
    result->billboard.billboard = render_batch_get_sprites(batch, count);
    result->billboard.sprite_mtx = sprite_mtx;

    return &result->billboard.billboard;
}

struct render_batch_billboard_element render_batch_get_sprites(struct render_batch *batch, int count)
{
    struct render_batch_billboard_element result;

    result.sprites = frame_malloc(batch->pool, count * sizeof(struct render_billboard_sprite));
    result.sprite_count = count;

    return result;
}

mat4x4 *render_batch_get_transform(struct render_batch *batch)
{
    return frame_malloc(batch->pool, sizeof(mat4x4));
}

T3DMat4FP *render_batch_get_transformfp(struct render_batch *batch)
{
    return UncachedAddr(frame_malloc(batch->pool, sizeof(T3DMat4FP)));
}

int render_batch_compare_element(struct render_batch *batch, uint16_t a_index, uint16_t b_index)
{
    struct render_batch_element *a = &batch->elements[a_index];
    struct render_batch_element *b = &batch->elements[b_index];

    if (a == b)
    {
        return 0;
    }

    if (!a->material && b->material)
    {
        return -b->material->sort_priority;

    }
    if (!b->material && a->material)
    {
        return a->material->sort_priority;

    }

    if ((a->material && b->material) && a->material->sort_priority != b->material->sort_priority)
    {
        return a->material->sort_priority - b->material->sort_priority;
    }

    if (a->material != b->material)
    {
        return (int)a->material - (int)b->material;
    }

    return a->type - b->type;
}

void render_batch_check_texture_scroll(int tile, struct material_tex *tex)
{
    if (!tex->sprite || (!tex->scroll_x && !tex->scroll_y))
    {
        return;
    }

    int w = tex->sprite->width << 2;
    int h = tex->sprite->height << 2;

    int x_offset = (int)(currtime_sec * tex->scroll_x * w) % w;
    int y_offset = (int)(currtime_sec * tex->scroll_y * h) % h;

    if (x_offset < 0)
    {
        x_offset += w;
    }

    if (y_offset < 0)
    {
        y_offset += h;
    }

    rdpq_set_tile_size_fx(
        tile,
        x_offset, y_offset,
        x_offset + w,
        y_offset + h);
}

static bool element_type_2d[] = {
    [RENDER_BATCH_MODEL] = false,
    [RENDER_BATCH_BILLBOARD] = true,
    [RENDER_BATCH_BILLBOARD_OLD] = true,
    [RENDER_BATCH_CALLBACK] = false,
};

void render_batch_execute(struct render_batch *batch, mat4x4 view_proj_matrix, T3DViewport *viewport)
{
    uint16_t order[RENDER_BATCH_MAX_SIZE];

    for (int i = 0; i < batch->element_count; ++i)
    {
        order[i] = i;
    }

    // used to scale billboard sprites
    float scale_x = sqrtf(
                        view_proj_matrix[0][0] * view_proj_matrix[0][0] +
                        view_proj_matrix[0][1] * view_proj_matrix[0][1] +
                        view_proj_matrix[0][2] * view_proj_matrix[0][2]) *
                    0.5f * 4;

    float scale_y = sqrtf(
                        view_proj_matrix[1][0] * view_proj_matrix[1][0] +
                        view_proj_matrix[1][1] * view_proj_matrix[1][1] +
                        view_proj_matrix[1][2] * view_proj_matrix[1][2]) *
                    0.5f * 4;

    sort_indices(order, batch->element_count, batch, (sort_compare)render_batch_compare_element);

    struct material *current_mat = 0;

    // rdpq_set_mode_standard();
    rdpq_mode_persp(true);
    rdpq_mode_zbuf(true, true);
    t3d_state_set_drawflags(T3D_FLAG_DEPTH | T3D_FLAG_SHADED | T3D_FLAG_TEXTURED);

    bool is_sprite_mode = false;
    bool z_write = true;
    bool z_read = true;

    for (int i = 0; i < batch->element_count; ++i)
    {
        int index = order[i];
        struct render_batch_element *element = &batch->elements[index];

        if (!element->material && current_mat != 0)
        {
            current_mat = 0;
            rdpq_mode_zbuf(true, true);
        }
        else if (current_mat != element->material)
        {
            if (element->material->block)
            {

                rspq_block_run(element->material->block);
            }

            render_batch_check_texture_scroll(TILE0, &element->material->tex0);
            render_batch_check_texture_scroll(TILE1, &element->material->tex1);

            bool need_z_write = (element->material->flags & MATERIAL_FLAGS_Z_WRITE) != 0;
            bool need_z_read = (element->material->flags & MATERIAL_FLAGS_Z_READ) != 0;

            if (need_z_write != z_write || need_z_read != z_read)
            {
                rdpq_mode_zbuf(need_z_read, need_z_write);
                z_write = need_z_write;
                z_read = need_z_read;
            }

            current_mat = element->material;
        }

        bool should_sprite_mode = element_type_2d[element->type];

        if (should_sprite_mode != is_sprite_mode)
        {
            if (should_sprite_mode)
            {
                rdpq_mode_persp(false);
            }
            else
            {
                rdpq_mode_zoverride(false, 0, 0);
                rdpq_mode_persp(true);
            }

            is_sprite_mode = should_sprite_mode;
        }
        // -------- Model Element ----------
        if (element->type == RENDER_BATCH_MODEL)
        {
            // Skip if no rspq block
            if (!element->model.block)
            {
                continue;
            }

            // Push transform if it exists
            if (element->model.transform)
            {
                t3d_matrix_push(element->model.transform);
            }

            // Run the rspq block rendering the model
            rspq_block_run(element->model.block);

            // Pop transform if it exists
            if (element->model.transform)
            {
                t3d_matrix_pop(1);
            }
        } // -------- Billboard Element ----------
        else if (element->type == RENDER_BATCH_BILLBOARD_OLD)
        {

            if (!element->material)
            {
                continue; // Skip if no material
            }
            // rdpq_mode_fog(0);
            // t3d_fog_set_enabled(false);

            // Loop through each sprite in the billboard
            for (int sprite_index = 0; sprite_index < element->billboard.billboard.sprite_count; ++sprite_index)
            {
                struct render_billboard_sprite sprite = element->billboard.billboard.sprites[sprite_index];

                // Transform sprite position to view projection space
                struct Vector4 transformed;
                struct Vector3 scaled;
                vector3Scale(&sprite.position, &scaled, SCENE_SCALE);
                matrixVec3Mul(view_proj_matrix, &scaled, &transformed);

                if (transformed.w < 0.0f)
                {
                    continue; // Skip if behind the camera
                }

                float wInv = 1.0f / transformed.w;

                // Calculate screen coordinates
                float x = (transformed.x * wInv + 1.0f) * 0.5f * 4.0f;
                float y = (-transformed.y * wInv + 1.0f) * 0.5f * 4.0f;
                float z = transformed.z * wInv * 0.5f + 0.5f;

                float size = sprite.radius * wInv * SCENE_SCALE;

                if (z < 0.0f || z > 1.0f)
                {
                    continue; // Skip if outside the depth range
                }

                // Override Z-buffer with sprite depth
                rdpq_mode_zoverride(true, z, 0);

                // Convert to screen space coordinates
                int screen_x = (int)(x * (viewport->size[0])) + viewport->offset[0] * 4;
                int screen_y = (int)(y * (viewport->size[1])) + viewport->offset[1] * 4;

                // Calculate half dimensions of the sprite on screen
                int half_screen_width = (int)(size * scale_x * viewport->size[0]);
                int half_screen_height = (int)(size * scale_y * viewport->size[1]);

                // Default image dimensions
                int image_w = 32;
                int image_h = 32;

                // Update image dimensions if material has a sprite
                if (element->material && element->material->tex0.sprite)
                {
                    image_w = element->material->tex0.sprite->width * 32;
                    image_h = element->material->tex0.sprite->height * 32;
                }

                // Set sprite color
                rdpq_set_prim_color(sprite.color);

                // Draw the sprite
                __rdpq_texture_rectangle_scaled_fx(
                    TILE0,
                    screen_x - half_screen_width,
                    screen_y - half_screen_height,
                    screen_x + half_screen_width,
                    screen_y + half_screen_height,
                    0,
                    0,
                    image_w,
                    image_h);
            }
        }
        else if (element->type == RENDER_BATCH_BILLBOARD)
        {

            if (!element->material)
            {
                continue; // Skip if no material
            }

            t3d_state_set_drawflags(T3D_FLAG_TEXTURED | T3D_FLAG_SHADED | T3D_FLAG_DEPTH);
 

            T3DMat4 billboardMat;

            // Loop through each sprite in the billboard
            for (int sprite_index = 0; sprite_index < element->billboard.billboard.sprite_count; ++sprite_index)
            {
                struct render_billboard_sprite sprite = element->billboard.billboard.sprites[sprite_index];

                // Transform sprite position to view projection space
                struct Vector4 transformed;
                struct Vector3 scaled;
                vector3Scale(&sprite.position, &scaled, SCENE_SCALE);
                matrixVec3Mul(view_proj_matrix, &scaled, &transformed);

                if (transformed.w < 0.0f)
                {
                    continue; // Skip if behind the camera
                }

                // Set sprite color
                rdpq_set_prim_color(sprite.color);

                float size = sprite.radius * SCENE_SCALE;

                t3d_mat4_identity(&billboardMat);
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        billboardMat.m[i][j] = view_proj_matrix[j][i]; // transpose rotation
                    }
                }
                billboardMat.m[0][3] = 0.0f;
                billboardMat.m[1][3] = 0.0f;
                billboardMat.m[2][3] = 0.0f;
                billboardMat.m[3][3] = 1.0f;

                billboardMat.m[0][0] *= size;
                billboardMat.m[0][1] *= size;
                billboardMat.m[0][2] *= size;
                billboardMat.m[1][0] *= size;
                billboardMat.m[1][1] *= size;
                billboardMat.m[1][2] *= size;
                billboardMat.m[2][0] *= size;
                billboardMat.m[2][1] *= size;
                billboardMat.m[2][2] *= size;

                billboardMat.m[3][0] = sprite.position.x * SCENE_SCALE;
                billboardMat.m[3][1] = sprite.position.y * SCENE_SCALE;
                billboardMat.m[3][2] = sprite.position.z * SCENE_SCALE;

                t3d_mat4_to_fixed(&element->billboard.sprite_mtx[sprite_index], &billboardMat);

                billboard_vertices[0] = (T3DVertPacked){
                    .posA = {-1, -1, 0},
                    .rgbaA = 0xFFFFFFFF,
                    .stA = {0, 2048},
                    .posB = {1, -1, 0},
                    .rgbaB = 0xFFFFFFFF,
                    .stB = {2048, 2028},
                };
                billboard_vertices[1] = (T3DVertPacked){
                    .posA = {1, 1, 0},
                    .rgbaA = 0xFFFFFFFF,
                    .stA = {2048, 0},
                    .posB = {-1, 1, 0},
                    .rgbaB = 0xFFFFFFFF,
                    .stB = {0, 0},
                };
                t3d_matrix_push(&element->billboard.sprite_mtx[sprite_index]);
                t3d_vert_load(billboard_vertices, 0, 4);
                t3d_matrix_pop(1);

                t3d_tri_draw(0, 1, 2);
                t3d_tri_draw(2, 3, 0);
                t3d_tri_sync();

                // free_uncached(vertices);
            }
        }
         // -------- Callback Element ----------
        else if (element->type == RENDER_BATCH_CALLBACK)
        {
            // Skip if no callback
            if (!element->callback.callback)
            {
                continue;
            }
            element->callback.callback(element->callback.data, batch);
        }
    }
}