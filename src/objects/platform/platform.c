#include "platform.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"


static struct dynamic_object_type platform_collision = {
    .minkowski_sum = box_minkowski_sum,
    .bounding_box = box_bounding_box,
    .data = {
        .box = {
            .half_size = {6.0f, 0.75f, 3.0f}
        }
    }
};

void platform_update(struct platform* platform){
    if (platform->transform.position.y <= 1)
    {
        platform->transform.position.y = 1;
        platform->collision.prev_position.y = 1;
    }
    platform->rot_x = 0.8f * frametime_sec;
    if(platform->rot_x > 360.0f) platform->rot_x -= 360.0f;
    quatRotateAxisEuler(&platform->transform.rotation, &gForward, platform->rot_x, &platform->transform.rotation);
}

void platform_init(struct platform* platform, struct platform_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&platform->transform);
    platform->transform.scale = (struct Vector3){6.0f, 0.5f, 3.0f};
    platform->transform.position = def->position;
    platform->rot_x = 0.0f;

    renderable_init(&platform->renderable, &platform->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&platform->renderable, 1.0f);


    dynamic_object_init(
        entity_id,
        &platform->collision,
        &platform_collision,
        COLLISION_LAYER_TANGIBLE,
        &platform->transform.position,
        &platform->transform.rotation,
        10.0f
    );

    platform->collision.center.y = platform_collision.data.box.half_size.y;

    platform->collision.has_gravity = 0;
    platform->collision.is_fixed = 1;

    update_add(platform, (update_callback)platform_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);
    collision_scene_add(&platform->collision);
}

void platform_destroy(struct platform* platform){
    render_scene_remove(&platform->renderable);
    renderable_destroy(&platform->renderable);
    collision_scene_remove(&platform->collision);
}
