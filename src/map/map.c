#include "map.h"

#include <libdragon.h>

#include "../math/vector2.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../render/defs.h"


void map_init(struct map* map) {
    transformInitIdentity(&map->transform);
    renderable_init(&map->renderable, &map->transform, "rom:/maps/bob_omb_battlefield/bob_map.t3dm");

    map->transform.position = (Vector3){{0,0,0}};
    map->transform.scale = (Vector3){{1.0f, 1.0f, 1.0f}};
    map->tileOffset = 0.0f;

    render_scene_add_callback(NULL, 0, render_scene_render_renderable, &map->renderable);
}

void map_destroy(struct map* map) {
    renderable_destroy(&map->renderable);
    render_scene_remove(&map->renderable);
}