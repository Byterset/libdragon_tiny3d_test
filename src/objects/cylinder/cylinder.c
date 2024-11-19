#include "cylinder.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/cylinder.h"
#include "../../collision/shapes/cylinder.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"


static struct physics_object_collision_data cylinder_collision = {
    .gjk_support_function = cylinder_support_function,
    .bounding_box_calculator = cylinder_bounding_box,
    .shape_data = {
        .cylinder = {
            .half_height = 2.5f,
            .radius = 6.0f
        }
    },
    .shape_type = COLLISION_SHAPE_CYLINDER,
};


void cylinder_init(struct cylinder* cylinder, struct cylinder_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&cylinder->transform);

    cylinder->transform.scale = (Vector3){{6.0f, 5.0f, 6.0f}};
    cylinder->transform.position = def->position;
    // quatRotateAxisEuler(&cylinder->transform.rotation, &gRight, T3D_DEG_TO_RAD(45.0f), &cylinder->transform.rotation);

    renderable_init(&cylinder->renderable, &cylinder->transform, "rom:/models/cylinder/cylinder.t3dm");

    render_scene_add_renderable(&cylinder->renderable, 6.0f);


    physics_object_init(
        entity_id,
        &cylinder->physics,
        &cylinder_collision,
        COLLISION_LAYER_TANGIBLE,
        &cylinder->transform.position,
        NULL,
        50.0f
    );

    // cylinder->collision.center_offset.y = cylinder_collision.shape_data.cylinder.half_height;
    cylinder->physics.center_offset.y = cylinder_collision.shape_data.cylinder.half_height;


    cylinder->physics.has_gravity = 1;
    cylinder->physics.is_fixed = 0;

    collision_scene_add(&cylinder->physics);
}

void cylinder_destroy(struct cylinder* cylinder){
    render_scene_remove(&cylinder->renderable);
    renderable_destroy(&cylinder->renderable);
    collision_scene_remove(&cylinder->physics);
}
