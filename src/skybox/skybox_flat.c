#include "skybox_flat.h"

#include <libdragon.h>

#include "../math/vector2.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../render/defs.h"
#include "../resource/sprite_cache.h"


void skybox_flat_update(void* data){

}

void skybox_flat_custom_render(void* data, struct render_batch* batch) {
    struct skybox_flat* skybox = (struct skybox_flat*)data;
    render_batch_add_skybox_flat(batch, &skybox->surface);
}

void skybox_flat_init(struct skybox_flat* skybox) {


    skybox->texture = sprite_cache_load("rom:/images/skybox/sky_seamless.sprite");
    // graphics_draw_sprite_trans
    //sprite_get_pixels -> gets the sprite pixels as a surface_t
    skybox->surface = sprite_get_pixels(skybox->texture);

    render_scene_add(0, 0, skybox_flat_custom_render, skybox);

}



void skybox_flat_destroy(struct skybox_flat* skybox) {
    sprite_cache_release(skybox->texture);

}