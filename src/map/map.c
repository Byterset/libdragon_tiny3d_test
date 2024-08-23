#include "map.h"

#include <libdragon.h>

#include "../math/vector2.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../entity/entity_id.h"
#include "../render/defs.h"


void map_init(struct map* map) {
    entity_id entity_id = entity_id_new();

    transformInitIdentity(&map->transform);
    renderable_init(&map->renderable, &map->transform, "rom:/models/map/map.t3dm");



    map->transform.position = (struct Vector3){0,0,-10};
    map->transform.scale = (struct Vector3){0.3f, 0.3f, 0.3f};

    render_scene_add_renderable(&map->renderable, 1.0f);


}

void map_destroy(struct map* map) {
    renderable_destroy(&map->renderable);
    render_scene_remove(&map->renderable);
}