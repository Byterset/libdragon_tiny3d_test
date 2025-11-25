#include "crate.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../collision/collision_scene.h"
#include "../../collision/shapes/box.h"
#include "../../collision/shapes/sphere.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

static struct physics_object_collision_data crate_collision = {
    BOX_COLLIDER(1.75f,1.75f,1.75f),
    .friction = 0.7f,
    .bounce = 0.0f
};

void crate_init(struct crate* crate, struct generic_object_pos_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&crate->transform);
    crate->transform.scale = (Vector3){{3.5f, 3.5f, 3.5f}};
    vector3Add(&crate->transform.position, &def->position, &crate->transform.position);

    renderable_init(&crate->renderable, &crate->transform, "rom:/models/crate/crate.t3dm");

    render_scene_add_renderable(&crate->renderable, 4.0f);


    physics_object_init(
        entity_id,
        &crate->physics,
        &crate_collision,
        COLLISION_LAYER_TANGIBLE,
        &crate->transform.position,
        &crate->transform.rotation,
        gZeroVec,
        100.0f
    );
    collision_scene_add(&crate->physics);
}

void crate_destroy(struct crate* crate){
    render_scene_remove(&crate->renderable);
    renderable_destroy(&crate->renderable);
    collision_scene_remove(&crate->physics);
}
