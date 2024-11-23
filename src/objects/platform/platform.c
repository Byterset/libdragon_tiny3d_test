#include "platform.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

#define ROTATION_DURATION 9.0f

static struct physics_object_collision_data platform_collision = {
    .gjk_support_function = box_support_function,
    .bounding_box_calculator = box_bounding_box,
    .shape_data = {
        .box = {
            .half_size = {{12.5f, 1.0f, 5.0f}}
        }
    },
    .shape_type = COLLISION_SHAPE_BOX,
};

void platform_update(struct platform* platform){
    platform->rot_elapsed_time += deltatime_sec;
    platform->rot_y = sinf((platform->rot_elapsed_time / ROTATION_DURATION) * 2.0f * PI) * T3D_DEG_TO_RAD(45);
    Quaternion rot;
    quatIdent(&rot);
    quatRotateAxisEuler(&rot, &gUp, T3D_DEG_TO_RAD(-45.0f), &rot);
    quatRotateAxisEuler(&rot, &gForward, platform->rot_y, &platform->transform.rotation);
    if(platform->rot_elapsed_time > ROTATION_DURATION){
        platform->rot_elapsed_time = 0.0f;
    }
}

void platform_init(struct platform* platform, struct platform_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&platform->transform);
    platform->transform.scale = (Vector3){{25.0f, 2.0f, 10.0f}};
    platform->transform.position = def->position;
    quatRotateAxisEuler(&platform->transform.rotation, &gUp, T3D_DEG_TO_RAD(-45.0f), &platform->transform.rotation);


    renderable_init(&platform->renderable, &platform->transform, "rom:/models/box/box.t3dm");

    render_scene_add_renderable(&platform->renderable, 25.0f);


    physics_object_init(
        entity_id,
        &platform->physics,
        &platform_collision,
        COLLISION_LAYER_TANGIBLE,
        &platform->transform.position,
        &platform->transform.rotation,
        10.0f
    );
    
    platform->physics.center_offset.y = platform_collision.shape_data.box.half_size.y;

    platform->physics.has_gravity = 0;
    platform->physics.is_fixed = 1;

    update_add(platform, (update_callback)platform_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);
    collision_scene_add(&platform->physics);
}

void platform_destroy(struct platform* platform){
    render_scene_remove(&platform->renderable);
    renderable_destroy(&platform->renderable);
    update_remove(platform);
    collision_scene_remove(&platform->physics);
}
