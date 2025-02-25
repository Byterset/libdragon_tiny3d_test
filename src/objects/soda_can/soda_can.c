#include "soda_can.h"

#include <libdragon.h>
#include "../../render/render_scene.h"
#include "../../time/time.h"
#include "../../entity/entity_id.h"
#include "../../render/defs.h"

void soda_can_update(struct soda_can *soda_can){
    quatRotateAxisEuler(&soda_can->transform.rotation, &gUp, T3D_DEG_TO_RAD(0.5f), &soda_can->transform.rotation);
}


void soda_can_init(struct soda_can* soda_can, struct generic_object_pos_definition* def){
    entity_id entity_id = entity_id_new();
    transformInitIdentity(&soda_can->transform);
    soda_can->transform.scale = (Vector3){{1.0f, 1.0f, 1.0f}};
    vector3Add(&soda_can->transform.position, &def->position, &soda_can->transform.position);

    renderable_init(&soda_can->renderable, &soda_can->transform, "rom:/models/soda_can/can.t3dm");

    render_scene_add_renderable(&soda_can->renderable, 2.0f);

    update_add(soda_can, (update_callback)soda_can_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);

}

void soda_can_destroy(struct soda_can* soda_can){
    render_scene_remove(&soda_can->renderable);
    renderable_destroy(&soda_can->renderable);
    update_remove(soda_can);
}
