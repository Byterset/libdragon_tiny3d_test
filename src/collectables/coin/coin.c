#include "coin.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/sphere.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

static struct physics_object_collision_data coin_collision = {
    .gjk_support_function = sphere_support_function,
    .bounding_box_calculator = sphere_bounding_box,
    .shape_data = {
        .sphere = {
            .radius = 0.5f
        }
    },
    .shape_type = COLLISION_SHAPE_SPHERE,
};

void coin_init(struct coin* coin, struct coin_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&coin->transform);

    coin->transform.scale = (struct Vector3){1.0f, 1.0f, 1.0f};
    vector3Copy(&def->position, &coin->transform.position);
    // quatRotateAxisEuler(&coin->transform.rotation, &gUp, T3D_DEG_TO_RAD(45.0f), &coin->transform.rotation);

    renderable_init(&coin->renderable, &coin->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&coin->renderable, 1.0f);


    physics_object_init(
        entity_id,
        &coin->physics,
        &coin_collision,
        COLLISION_LAYER_COLLECTABLES,
        &coin->transform.position,
        NULL,
        1.0f
    );

    coin->physics.collision_group = COLLISION_GROUP_COLLECTABLE;
    coin->physics.center_offset.y = 0.0f;
    coin->physics.has_gravity = 0;
    coin->physics.is_fixed = 1;
    coin->physics.is_trigger = 1;

    collision_scene_add(&coin->physics);
}

void coin_destroy(struct coin* coin){
    render_scene_remove(&coin->renderable);
    renderable_destroy(&coin->renderable);
    collision_scene_remove(&coin->physics);
}
