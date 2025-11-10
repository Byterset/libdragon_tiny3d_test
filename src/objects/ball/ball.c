#include "ball.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../collision/shapes/sphere.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

static struct physics_object_collision_data ball_collision = {
    .gjk_support_function = sphere_support_function,
    .bounding_box_calculator = sphere_bounding_box,
    .inertia_calculator = sphere_inertia_tensor,
    .shape_data = {
        .sphere = {
            .radius = 2.0f
        }
    },
    .shape_type = COLLISION_SHAPE_SPHERE,
    .friction = 0.6,
    .bounce = 0.4
};

void ball_init(struct ball* ball, struct generic_object_pos_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&ball->transform);
    ball->transform.scale = (Vector3){{4.0f, 4.0f, 4.0f}};
    vector3Add(&ball->transform.position, &def->position, &ball->transform.position);

    renderable_init(&ball->renderable, &ball->transform, "rom:/models/ball/ball.t3dm");

    render_scene_add_renderable(&ball->renderable, 4.0f);


    physics_object_init(
        entity_id,
        &ball->physics,
        &ball_collision,
        COLLISION_LAYER_TANGIBLE,
        &ball->transform.position,
        &ball->transform.rotation,
        gZeroVec,
        100.0f
    );
    collision_scene_add(&ball->physics);
}

void ball_destroy(struct ball* ball){
    render_scene_remove(&ball->renderable);
    renderable_destroy(&ball->renderable);
    collision_scene_remove(&ball->physics);
}
