#include "box.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../collision/shapes/sphere.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

static struct physics_object_collision_data box_collision = {
    .gjk_support_function = box_support_function,
    .bounding_box_calculator = box_bounding_box,
    .shape_data = {
        .box = {
            .half_size = {1.0f, 1.0f, 1.0f}
        }
    },
    .shape_type = COLLISION_SHAPE_BOX,
};

void box_init(struct box* box, struct box_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&box->transform);

    box->transform.scale = (Vector3){2.0f, 2.0f, 2.0f};
    vector3Add(&box->transform.position, &def->position, &box->transform.position);
    // quatRotateAxisEuler(&box->transform.rotation, &gUp, T3D_DEG_TO_RAD(45.0f), &box->transform.rotation);

    renderable_init(&box->renderable, &box->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&box->renderable, 1.0f);


    physics_object_init(
        entity_id,
        &box->physics,
        &box_collision,
        COLLISION_LAYER_TANGIBLE,
        &box->transform.position,
        &box->transform.rotation,
        10.0f
    );
    box->physics.center_offset.y = box_collision.shape_data.box.half_size.y;
    box->physics.has_gravity = 1;
    box->physics.collision->friction = 0.1f;

    collision_scene_add(&box->physics);
}

void box_destroy(struct box* box){
    render_scene_remove(&box->renderable);
    renderable_destroy(&box->renderable);
    collision_scene_remove(&box->physics);
}
