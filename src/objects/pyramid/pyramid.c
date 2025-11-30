#include "pyramid.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/pyramid.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"


static struct physics_object_collision_data pyramid_collision = {
    PYRAMID_COLLIDER(5.0f, 5.0f, 4.0f),
    .friction = 0.4
};


void pyramid_init(struct pyramid* pyramid, struct generic_object_pos_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&pyramid->transform);
    pyramid->transform.scale = (Vector3){{7.0f, 8.0f, 7.0f}};
    pyramid->transform.position = def->position;
    quatRotateAxisEuler(&pyramid->transform.rotation, &gRight, T3D_DEG_TO_RAD(0.0f), &pyramid->transform.rotation);
    pyramid->transform.position.y += 2.0f;

    renderable_init(&pyramid->renderable, &pyramid->transform, "rom:/models/pyramid/pyramid.t3dm");

    render_scene_add_renderable(&pyramid->renderable, 14.0f);

    physics_object_init(
        entity_id,
        &pyramid->physics,
        &pyramid_collision,
        COLLISION_LAYER_TANGIBLE,
        &pyramid->transform.position,
        &pyramid->transform.rotation,
        (Vector3){{0, pyramid_collision.shape_data.pyramid.half_height, 0}},
        120.0f);

    pyramid->physics.has_gravity = true;
    pyramid->physics.is_kinematic = false;
    pyramid->physics.angular_damping = 0.03f;
    // pyramid->physics.constraints |= CONSTRAINTS_FREEZE_POSITION_ALL;

    collision_scene_add(&pyramid->physics);
}

void pyramid_destroy(struct pyramid* pyramid){
    render_scene_remove(&pyramid->renderable);
    renderable_destroy(&pyramid->renderable);
    collision_scene_remove(&pyramid->physics);
}
