#include "platform.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"


static struct dynamic_object_type platform_collision = {
    .minkowsi_sum = box_minkowski_sum,
    .bounding_box = box_bounding_box,
    .data = {
        .box = {
            .half_size = {6.0f, 1.0f, 3.0f}
        }
    }
};

void platform_update(struct platform* platform){
    if (platform->transform.position.y <= 1)
    {
        platform->transform.position.y = 1;
        platform->collision.prev_position.y = 1;
    }
}

void platform_init(struct platform* platform, struct platform_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&platform->transform);
    platform->transform.scale = (struct Vector3){6.0f, 1.0f, 3.0f};
    platform->transform.position = def->position;

    renderable_init(&platform->renderable, &platform->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&platform->renderable, 1.0f);


    dynamic_object_init(
        entity_id,
        &platform->collision,
        &platform_collision,
        COLLISION_LAYER_TANGIBLE,
        &platform->transform.position,
        NULL,
        10.0f
    );

    // platform->collision.center.y = platform_collision.data.box.half_size.y;

    platform->collision.has_gravity = false;
    platform->collision.is_fixed = true;

    update_add(platform, (update_callback)platform_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);
    collision_scene_add(&platform->collision);
}

void platform_destroy(struct platform* platform){
    render_scene_remove(&platform->renderable);
    renderable_destroy(&platform->renderable);
    collision_scene_remove(&platform->collision);
}