#include "cone.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/cone.h"
#include "../../collision/shapes/cylinder.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"


static struct physics_object_collision_data cone_collision = {
    .gjk_support_function = cone_support_function,
    .bounding_box_calculator = cone_bounding_box,
    .shape_data = {
        .cone = {
            .half_height = 2.5f,
            .radius = 7.0f
        }
    },
    .shape_type = COLLISION_SHAPE_CONE,
};


void cone_init(struct cone* cone, struct cone_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&cone->transform);

    cone->transform.scale = (Vector3){7.0f, 5.0f, 7.0f};
    cone->transform.position = def->position;
    quatRotateAxisEuler(&cone->transform.rotation, &gRight, T3D_DEG_TO_RAD(0.0f), &cone->transform.rotation);
    // cone->transform.position.y += 4.5f;

    renderable_init(&cone->renderable, &cone->transform, "rom:/models/cone/cone.t3dm");

    render_scene_add_renderable(&cone->renderable, 1.0f);


    physics_object_init(
        entity_id,
        &cone->physics,
        &cone_collision,
        COLLISION_LAYER_TANGIBLE,
        &cone->transform.position,
        &cone->transform.rotation,
        10.0f
    );

    // cone->collision.center_offset.y = cone_collision.shape_data.cylinder.half_height;
    cone->physics.center_offset.y = cone_collision.shape_data.cone.half_height;


    cone->physics.has_gravity = 0;
    cone->physics.is_fixed = 1;

    collision_scene_add(&cone->physics);
}

void cone_destroy(struct cone* cone){
    render_scene_remove(&cone->renderable);
    renderable_destroy(&cone->renderable);
    collision_scene_remove(&cone->physics);
}
