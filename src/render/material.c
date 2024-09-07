#include "material.h"

#include <t3d/t3d.h>
#include "../resource/sprite_cache.h"

/// @brief initialize a material with default values
/// @param material 
void material_init(struct material* material) {
    material->block = 0;

    material->tex0.sprite = NULL;

    material->tex1.sprite = NULL;

    material->sort_priority = SORT_PRIORITY_OPAQUE;

    material->palette.tlut = 0;
    material->palette.idx = 0;
    material->palette.size = 0;
    material->flags = 0;
}

/// @brief free the momory of a material object. Will free the associated textures, palette and rspq_block
/// @param material pointer to the material to be destroyed
void material_destroy(struct material* material) {
    if (material->tex0.sprite) {
        sprite_cache_release(material->tex0.sprite);
    }
    if (material->tex1.sprite) {
        sprite_cache_release(material->tex1.sprite);
    }
    rspq_block_free(material->block);
    free(material->palette.tlut);
    material->palette.tlut = 0;
}

// MATR
#define EXPECTED_HEADER 0x4D415452

#define COMMAND_EOF         0x00
#define COMMAND_COMBINE     0x01
#define COMMAND_BLEND       0x02
#define COMMAND_ENV         0x03
#define COMMAND_PRIM        0x04
#define COMMAND_BLEND_COLOR 0x05
#define COMMAND_FLAGS       0x06
#define COMMAND_PALETTE     0x07
#define COMMAND_UV_GEN      0x08

struct text_axis {
    float translate;
    int scale_log;
    float repeats;
    bool mirror;
};

void material_load_tex_axis(struct text_axis* axis, FILE* file) {
    int16_t translate;
    fread(&translate, 2, 1, file);
    axis->translate = translate * (1.0f / 32.0f);

    int8_t scale_log;
    fread(&scale_log, 1, 1, file);
    axis->scale_log = scale_log;

    uint16_t repeats;
    fread(&repeats, 2, 1, file);
    axis->repeats = repeats & 0x7FFF;

    axis->mirror = (repeats & 0x8000) != 0;
}

static GLenum material_filter_modes[] = {
    GL_NEAREST,
    GL_LINEAR,
};

/// @brief load a texture from a sprite file via the sprite cache and set the texture parameters.
/// First the filename is read from the file stream and the sprite loaded from the read filename, then the texture parameters are read and set.
/// @param tex the target texture
/// @param file the material file stream to read from
/// @param create_texture 
void material_load_tex(struct material_tex* tex, FILE* file) {
    uint8_t filename_len;
    fread(&filename_len, 1, 1, file);

    if (filename_len == 0) {
        return;
    }

    char filename[filename_len + 1];
    fread(filename, 1, filename_len, file);
    filename[filename_len] = '\0';

    uint16_t tmem_addr;
    fread(&tmem_addr, 2, 1, file);
    tex->params.tmem_addr = tmem_addr;
    uint8_t palette;
    fread(&palette, 1, 1, file);
    tex->params.palette = palette;

    material_load_tex_axis((struct text_axis*)&tex->params.s, file);
    material_load_tex_axis((struct text_axis*)&tex->params.t, file);

    fread(&tex->scroll_x, sizeof(float), 1, file);
    fread(&tex->scroll_y, sizeof(float), 1, file);

    tex->sprite = sprite_cache_load(filename);

    uint8_t mag_filter;
    fread(&mag_filter, 1, 1, file);
    uint8_t min_filter;
    fread(&min_filter, 1, 1, file);
}

/// @brief Read a custom material from a file stream and create a material object with a rspq_block that contains the material commands
/// @param into the target material
/// @param material_file the file stream to read from
void material_load(struct material* into, FILE* material_file) {
    // read header and make sure it is equal to the expected header
    int header;
    fread(&header, 1, 4, material_file);
    assert(header == EXPECTED_HEADER);

    bool has_more = true;

    //initialize an empty material
    material_init(into);

    // load the given textures and their parameters
    material_load_tex(&into->tex0, material_file);
    material_load_tex(&into->tex1, material_file);


    // record the rspq_block for applying the material
    rspq_block_begin();

    // check if the material is using auto layout tmem
    bool autoLayoutTMem = into->tex1.sprite != 0 && into->tex1.params.tmem_addr == 0;
    if (autoLayoutTMem) {
        rdpq_tex_multi_begin();
    }

    // upload the textures to the RDP texture memory
    if (into->tex0.sprite) {
        rdpq_sprite_upload(TILE0, into->tex0.sprite, &into->tex0.params);
    }
    if (into->tex1.sprite) {
        rdpq_sprite_upload(TILE1, into->tex1.sprite, &into->tex1.params);
    }

    if (autoLayoutTMem) {
        rdpq_tex_multi_end();
    }

    rdpq_mode_begin();
    //read the material commands from the file stream and apply them
    while (has_more) {
        uint8_t nextCommand;
        fread(&nextCommand, 1, 1, material_file);

        switch (nextCommand) {
            case COMMAND_EOF:
                has_more = false;
                break;
            case COMMAND_COMBINE:
                {
                    rdpq_combiner_t combineMode;
                    fread(&combineMode, sizeof(rdpq_combiner_t), 1, material_file);
                    rdpq_mode_combiner(combineMode);
                }
                break;
            case COMMAND_BLEND:
                {
                    rdpq_blender_t blendMode;
                    fread(&blendMode, sizeof(rdpq_blender_t), 1, material_file);
                    rdpq_mode_blender(blendMode & SOM_BLEND_MASK);  

                    if (blendMode & SOM_Z_COMPARE) {
                        into->flags |= MATERIAL_FLAGS_Z_READ;
                    }                  

                    if (blendMode & SOM_Z_WRITE) {
                        into->flags |= MATERIAL_FLAGS_Z_WRITE;
                    }

                    if ((blendMode & SOM_ALPHACOMPARE_MASK) != 0) {
                        if ((blendMode & SOM_ALPHACOMPARE_MASK) == SOM_ALPHACOMPARE_THRESHOLD) {
                            rdpq_mode_alphacompare(128);
                        } else {
                            rdpq_mode_alphacompare(-1);
                        }
                    } else {
                        rdpq_mode_alphacompare(0);
                    }

                    // TODO check when the zmode is decal
                    // into->sort_priority = SORT_PRIORITY_DECAL;

                    if ((blendMode & SOM_Z_WRITE) == 0) {
                        into->sort_priority = SORT_PRIORITY_TRANSPARENT;
                    }
                }
                break;
            case COMMAND_ENV:
                {
                    color_t color;
                    fread(&color, sizeof(color_t), 1, material_file);
                    rdpq_set_env_color(color);
                }
                break;
            case COMMAND_PRIM:
                {
                    color_t color;
                    fread(&color, sizeof(color_t), 1, material_file);
                    rdpq_set_prim_color(color);
                }
                break;
            case COMMAND_BLEND_COLOR:
                {
                    color_t color;
                    fread(&color, sizeof(color_t), 1, material_file);
                    rdpq_set_blend_color(color);
                }
                break;
            case COMMAND_FLAGS:
                {
                    uint16_t flags;
                    fread(&flags, 2, 1, material_file);
                    t3d_state_set_drawflags(flags);
                }
                break;
            case COMMAND_PALETTE:
                {
                    fread(&into->palette.idx, 2, 1, material_file);
                    fread(&into->palette.size, 2, 1, material_file);
                    into->palette.tlut = malloc(sizeof(uint16_t) * into->palette.size);
                    rdpq_tex_upload_tlut(into->palette.tlut, into->palette.idx, into->palette.size);
                }
                break;
            case COMMAND_UV_GEN:
                {
                    uint8_t fn;
                    fread(&fn, 1, 1, material_file);

                    switch (fn) {
                        case T3D_VERTEX_FX_NONE:
                            t3d_state_set_vertex_fx(T3D_VERTEX_FX_NONE, 0, 0);
                            break;
                        case T3D_VERTEX_FX_SPHERICAL_UV:
                            t3d_state_set_vertex_fx(T3D_VERTEX_FX_SPHERICAL_UV, into->tex0.sprite->width, into->tex0.sprite->height);
                            break;
                    }
                }
                break;
        }
    }

    rdpq_mode_end();

    into->block = rspq_block_end();
}

void material_release(struct material* material) {
    material_destroy(material);
}