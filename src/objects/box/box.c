#include "box.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../collision/shapes/sphere.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

static struct dynamic_object_type box_collision = {
    .minkowski_sum = box_minkowski_sum,
    .bounding_box = box_bounding_box,
    .data = {
        .box = {
            .half_size = {1.0f, 1.0f, 1.0f}
        }
    },
    .type = DYNAMIC_OBJECT_TYPE_BOX,
};

// static struct dynamic_object_type box_collision = {
//     .minkowski_sum = sphere_minkowski_sum,
//     .bounding_box = sphere_bounding_box,
//     .data = {
//         .sphere = {
//             .radius = 1.0f
//         }
//     },
//     .type = DYNAMIC_OBJECT_TYPE_SPHERE,
// };

void box_update(struct box* box){
    if (box->transform.position.y <= 0)
    {
        box->transform.position.y = 0;
        box->collision.prev_position.y = 0;
    }

}

void box_init(struct box* box, struct box_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&box->transform);

    box->transform.scale = (struct Vector3){2.0f, 2.0f, 2.0f};
    box->transform.position = def->position;
    quatRotateAxisEuler(&box->transform.rotation, &gUp, 45.0f, &box->transform.rotation);
    box->look_direction = (struct Vector2){1.0f, 0.0f};

    renderable_init(&box->renderable, &box->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&box->renderable, 1.0f);


    dynamic_object_init(
        entity_id,
        &box->collision,
        &box_collision,
        COLLISION_LAYER_TANGIBLE,
        &box->transform.position,
        &box->transform.rotation,
        10.0f
    );

    box->collision.center.y = box_collision.data.box.half_size.y;
    // box->collision.center.y = box_collision.data.sphere.radius;


    box->collision.has_gravity = 1;

    update_add(box, (update_callback)box_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);
    collision_scene_add(&box->collision);
}

void box_destroy(struct box* box){
    render_scene_remove(&box->renderable);
    renderable_destroy(&box->renderable);
    collision_scene_remove(&box->collision);
}
