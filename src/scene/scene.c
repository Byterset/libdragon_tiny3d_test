#include "scene.h"

#include "../render/render_batch.h"

struct scene* current_scene;

static char next_scene_name[64];
static char next_entrance_name[16];

void scene_render(void* data, struct render_batch* batch) {
    struct scene* scene = (struct scene*)data;

    for (int i = 0; i < scene->static_entity_count; ++i) {
        render_batch_add_t3dmodel(batch, &scene->static_entities[i].model, NULL);
    }
}

void scene_update(void* data) {
    struct scene* scene = (struct scene*)data;

    Vector3 player_center = scene->player.transform.position;
    player_center.y += scene->player.physics.center_offset.y;

    for (int i = 0; i < scene->loading_zone_count; i += 1) {
        if (AABBContainsPoint(&scene->loading_zones[i].bounding_box, &player_center)) {
            scene_queue_next(scene->loading_zones[i].scene_name);
        }
    }
}

void scene_queue_next(char* scene_name) {
    char* curr = scene_name;
    char* out = next_scene_name;
    while (*curr && *curr != '#') {
        *out++ = *curr++;
    }

    *out = '\0';

    out = next_entrance_name;

    if (!*curr) {
        next_entrance_name[0] = '\0';
        return;
    }

    // skip the bound symbol
    ++curr;

    while ((*out++ = *curr++));
}

void scene_clear_next() {
    next_scene_name[0] = '\0';
}

bool scene_has_next() {
    return next_scene_name[0] != 0;
}

char* scene_get_next() {
    return next_scene_name;
}

char* scene_get_next_entry() {
    return next_entrance_name;
}