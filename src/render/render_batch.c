#include "render_batch.h"

#include "../util/sort.h"
#include "../time/time.h"
#include "../math/mathf.h"
#include "../math/math.h"
#include "material.h"
#include "defs.h"

T3DVertPacked billboard_vertices[2];

void render_batch_init(struct render_batch *batch, Transform *camera_transform, struct frame_memory_pool *pool)
{
    batch->element_count = 0;
    batch->pool = pool;
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

void render_batch_add_t3dmodel(struct render_batch *batch, struct model *model, T3DMat4FP *transform)
{
    struct render_batch_element *element = render_batch_add_init(batch);

    if (!element)
    {
        return;
    }

    element->type = RENDER_BATCH_MODEL;
    element->model.block = model->t3d_model->userBlock;
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

struct render_batch_billboard_element *render_batch_add_particles(struct render_batch *batch, struct material *material, int count)
{
    struct render_batch_element *result = render_batch_add_init(batch);

    result->type = RENDER_BATCH_BILLBOARD;
    result->material = material;
    result->billboard = render_batch_get_sprites(batch, count);

    return &result->billboard;
}

void render_batch_add_equidistant(struct render_batch* batch, rspq_block_t* block){
    struct render_batch_element* element = render_batch_add_init(batch);

    if (!element)
    {
        return;
    }
    

    element->type = RENDER_BATCH_EQUIDISTANT;
    element->material = NULL;
    element->model.block = block;
    element->model.transform = NULL;
}

void render_batch_add_skybox_flat(struct render_batch* batch, surface_t* surface){
    struct render_batch_element* element = render_batch_add_init(batch);

    if (!element)
    {
        return;
    }
    
    element->type = RENDER_BATCH_SKYBOX;
    element->material = NULL;
    element->skybox.surface = surface;
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

/**
 * @brief Compares two elements in a render batch.
 *
 * This function compares two elements in a render batch based on their materials and types.
 * It is used to determine the order of elements for rendering.
 *
 * @param batch Pointer to the render batch containing the elements.
 * @param a_index Index of the first element to compare.
 * @param b_index Index of the second element to compare.
 * @return An integer less than, equal to, or greater than zero if the first element is considered
 *         to be respectively less than, equal to, or greater than the second element.
 *
 * The comparison is performed as follows:
 * - If both elements are the same, return 0.
 * - If the first element has no material and the second element has a material, return the negative sort priority of the second element's material.
 * - If the second element has no material and the first element has a material, return the sort priority of the first element's material.
 * - If both elements have materials and their sort priorities differ, return the difference between their sort priorities.
 * - If both elements have materials but they are different, return the difference between their material pointers.
 * - If all previous checks are equal, return the difference between their types.
 */
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
    [RENDER_BATCH_SKYBOX] = true,
    [RENDER_BATCH_MODEL] = false,
    [RENDER_BATCH_BILLBOARD] = true,
    [RENDER_BATCH_CALLBACK] = false,
    [RENDER_BATCH_EQUIDISTANT] = false,
    
};

void render_batch_execute(struct render_batch *batch, mat4x4 view_proj_matrix, T3DViewport *viewport, struct render_fog_params *fog)
{
    uint16_t order[RENDER_BATCH_MAX_SIZE];

    for (int i = 0; i < batch->element_count; ++i)
    {
        order[i] = i;
    }

    // used to scale billboard sprites
    float billboard_scale_x = sqrtf(
                        view_proj_matrix[0][0] * view_proj_matrix[0][0] +
                        view_proj_matrix[0][1] * view_proj_matrix[0][1] +
                        view_proj_matrix[0][2] * view_proj_matrix[0][2]) *
                    0.5f * 4;

    float billboard_scale_y = sqrtf(
                        view_proj_matrix[1][0] * view_proj_matrix[1][0] +
                        view_proj_matrix[1][1] * view_proj_matrix[1][1] +
                        view_proj_matrix[1][2] * view_proj_matrix[1][2]) *
                    0.5f * 4;

    sort_indices(order, batch->element_count, batch, (sort_compare)render_batch_compare_element);

    bool is_sprite_mode = false;
    bool z_write = true;
    bool z_read = true;

    for (int i = 0; i < batch->element_count; ++i)
    {
        int index = order[i];
        struct render_batch_element *element = &batch->elements[index];

        bool should_sprite_mode = element_type_2d[element->type];

        if (should_sprite_mode != is_sprite_mode)
        {
            if (should_sprite_mode)
            {
                rdpq_set_mode_standard();
                rdpq_mode_persp(false);
            }
            else
            {
                if(fog && fog->enabled){
                    rdpq_mode_fog(RDPQ_FOG_STANDARD);
                    rdpq_set_fog_color(fog->color);
                    t3d_fog_set_enabled(true);
                    t3d_fog_set_range(fog->start, fog->end);
                } else {
                    t3d_fog_set_enabled(false);
                }

                rdpq_mode_zoverride(false, 0, 0);
                rdpq_mode_persp(true);
            }

            is_sprite_mode = should_sprite_mode;
        }
        // -------- Model Element ----------
        if (element->type == RENDER_BATCH_MODEL)
        {
            rdpq_mode_persp(true);
            rdpq_mode_zbuf(true, true);
            t3d_state_set_drawflags(T3D_FLAG_DEPTH | T3D_FLAG_SHADED | T3D_FLAG_TEXTURED);
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
        else if (element->type == RENDER_BATCH_BILLBOARD)
        {

            if (!element->material || !element->material->block)
            {
                continue; // Skip if no material since that indicates there is also no texture to be rendered
            }

            rspq_block_run(element->material->block);

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

            // Loop through each sprite in the billboard
            for (int sprite_index = 0; sprite_index < element->billboard.sprite_count; ++sprite_index)
            {
                struct render_billboard_sprite sprite = element->billboard.sprites[sprite_index];

                // Transform sprite position to view projection space
                Vector4 transformed;
                Vector3 scaled;
                vector3Scale(&sprite.position, &scaled, SCENE_SCALE);
                matrixVec3Mul(view_proj_matrix, &scaled, &transformed);

                // w is the homogeneous coordinate, if it is less than 0 the point is behind the camera
                
                if (transformed.w < 0.0f)
                {
                    continue; // Skip if behind the camera
                }

                // the inverse of the homogeneous coordinate is used to calculate the screen space coordinates
                float wInv = 1.0f / transformed.w;

                // Calculate screen space coordinates
                float x = (transformed.x * wInv + 1.0f) * 0.5f * 4.0f;
                float y = (-transformed.y * wInv + 1.0f) * 0.5f * 4.0f;
                float z = (transformed.z * wInv + 1.0f) * 0.5f; // Corrected z calculation
                float billboard_size = sprite.radius * wInv * SCENE_SCALE;

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
                int half_screen_width = (int)(billboard_size * billboard_scale_x * viewport->size[0]);
                int half_screen_height = (int)(billboard_size * billboard_scale_y * viewport->size[1]);

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
        // skybox rendered as a physical object
        else if (element->type == RENDER_BATCH_EQUIDISTANT){
            if(!element->model.block){
                continue;
            }
            rdpq_mode_persp(true);
            rdpq_mode_zbuf(true, true);
            t3d_state_set_drawflags(T3D_FLAG_DEPTH | T3D_FLAG_SHADED | T3D_FLAG_TEXTURED);
            rdpq_mode_zoverride(true, 1, 0);
            T3DMat4FP *mtxfp = render_batch_get_transformfp(batch);

            if (!mtxfp)
            {
                return;
            }

            T3DMat4 mtx;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    mtx.m[i][j] = viewport->matCamera.m[i][j];
                }
            }

            mtx.m[3][0] = 0;
            mtx.m[3][1] = 0;
            mtx.m[3][2] = 0;

            t3d_mat4_scale(&mtx, SCENE_SCALE, SCENE_SCALE, SCENE_SCALE);

            t3d_mat4_to_fixed_3x4(mtxfp, &mtx);
            t3d_matrix_set(mtxfp, false);
            rspq_block_run(element->model.block);
        }
        // -------- Skybox Flat Element ----------
        else if (element->type == RENDER_BATCH_SKYBOX)
        {
            if (!element->skybox.surface )
            {
                continue;
            }

            // Ensure the skybox texture is at least as large as the display otherwise the wrap will not work
            assert(element->skybox.surface->width >= display_get_width() && element->skybox.surface->height >= display_get_height());

            // Calculate forward vector from camera matrix
            Vector3 forward = (Vector3){{viewport->matCamera.m[0][2], viewport->matCamera.m[1][2], viewport->matCamera.m[2][2]}};
            // Calculate yaw and pitch from forward vector (negative to reverse the direction or rotation)
            float inv_yaw = -atan2f(forward.x, forward.z);
            float pitch = asinf(forward.y);

            if (pitch >= -(0.001) && pitch <= 0.001)
            {
                pitch = 0; // Prevent floating point errors from making the skybox flicker up and down ever so slightly at rest
            }

            // basically what the following does:
            // when using a 960:720 texture the window into the tex will be the size of the display
            // for other texture sizes the window will be scaled accordingly
            float ideal_tex_width = 960.0f;
            float ideal_tex_height = 720.0f;

            float scaling_height = ideal_tex_height / element->skybox.surface->height;
            float scaling_width = ideal_tex_width / element->skybox.surface->width;

            // not sure how this looks when using high res mode
            int section_width = display_get_width()/scaling_width;
            int section_height = display_get_height()/scaling_height;

            // Normalize yaw and pitch between 0 and 1
            inv_yaw = ((inv_yaw + PI) / TWO_PI); // Normalize yaw from [-π, π] to [0, 1]
            pitch = (pitch + HALF_PI) / PI; // Normalize pitch from [-π/2, π/2] to [0, 1]

            int texOffsetX = inv_yaw * element->skybox.surface->width - (section_width / 2);
            int texOffsetY = pitch * element->skybox.surface->height - (section_height / 2);

            texOffsetX = texOffsetX % element->skybox.surface->width;
            texOffsetY = texOffsetY % element->skybox.surface->height;


            texOffsetX = texOffsetX < 0 ? element->skybox.surface->width + texOffsetX : texOffsetX;
            // since we are not wrapping the image in the vertical direction, we need to clamp the offset
            texOffsetY = clampi(texOffsetY, 0, element->skybox.surface->height - 1 - section_height);

            rdpq_set_mode_standard();
            rdpq_mode_zoverride(true, 1, 0);
            // color_t tint = RGBA32(255, 255, 255, 128);
            // float brightness = 1.0f;
            // tint.r = (uint8_t)(tint.r * brightness);
            // tint.g = (uint8_t)(tint.g * brightness);
            // tint.b = (uint8_t)(tint.b * brightness);
            // rdpq_set_prim_color(tint);
            // rdpq_combiner_t cc = RDPQ_COMBINER1((PRIM,0,TEX0,0),    (PRIM,0,TEX0,0));
            // rdpq_blender_t blend = RDPQ_BLENDER((IN_RGB, IN_ALPHA, MEMORY_RGB, INV_MUX_ALPHA));
            // rdpq_mode_blender(blend);
            // rdpq_mode_combiner(cc);

            // if the window is within the bounds of the texture, just blit it
            if (texOffsetX + section_width < element->skybox.surface->width)
            {

                rdpq_tex_blit(element->skybox.surface, 0, 0, &(rdpq_blitparms_t){
                                                                    .s0 = texOffsetX,
                                                                    .t0 = texOffsetY,
                                                                    .scale_x = scaling_width,
                                                                    .scale_y = scaling_height,
                                                                    .width = section_width,
                                                                    .height = section_height,
                                                                });
            }
            // Split the blit into two parts to achieve a wrap around if the window overlapts the edge of the texture
            else
            {
                int first_width = element->skybox.surface->width - 1 - texOffsetX;
                int second_width = section_width - first_width;

                // Left side (from x-offset until the end of the texture)
                if(first_width > 0){
                    rdpq_tex_blit(element->skybox.surface, 0, 0, &(rdpq_blitparms_t){
                                                                    .s0 = texOffsetX,
                                                                    .t0 = texOffsetY,
                                                                    .scale_x = scaling_width,
                                                                    .scale_y = scaling_height,
                                                                    .width = first_width,
                                                                    .height = section_height
                                                                });
                }

                // Right side (from the start of the texture until the remaining width)
                if(second_width > 0){
                    rdpq_tex_blit(element->skybox.surface, first_width * scaling_width, 0, &(rdpq_blitparms_t){
                                                                    .s0 = 0,
                                                                    .t0 = texOffsetY,
                                                                    .scale_x = scaling_width,
                                                                    .scale_y = scaling_height,
                                                                    .width = second_width,
                                                                    .height = section_height
                                                                });
                }
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