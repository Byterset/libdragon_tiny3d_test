#include "box.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

static struct dynamic_object_type box_collision = {
    .minkowsi_sum = box_minkowski_sum,
    .bounding_box = box_bounding_box,
    .data = {
        .box = {
            .half_size = {1.0f, 1.0f, 1.0f}
        }
    }
};

void box_update(struct box* box){
    if (box->transform.position.y <= 1)
    {
        box->transform.position.y = 1;
        box->collision.velocity.y = box->collision.velocity.y <= 0 ? 0 : box->collision.velocity.y;
    }
}

void box_init(struct box* box, struct Vector3 position){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&box->transform);
    box->transform.scale = (struct Vector3){1.0f, 1.0f, 1.0f};
    box->transform.position = position;

    renderable_init(&box->renderable, &box->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&box->renderable, 1.0f);


    dynamic_object_init(
        entity_id,
        &box->collision,
        &box_collision,
        COLLISION_LAYER_TANGIBLE,
        &box->transform.position,
        &box->look_direction
    );

    box->collision.center.y = box_collision.data.box.half_size.y;

    box->collision.has_gravity = true;

    update_add(box, (update_callback)box_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);
    collision_scene_add(&box->collision);
}

void box_destroy(struct box* box){
    render_scene_remove(&box->renderable);
    renderable_destroy(&box->renderable);
    collision_scene_remove(&box->collision);
}
