#include "map.h"

#include <libdragon.h>

#include "../math/vector2.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../render/defs.h"


// Tests for the map - texture scrolling and rotating update

void tile_scroll(void* userData, rdpq_texparms_t *tileParams, rdpq_tile_t tile) {
  float offset = *(float*)userData;
  if(tile == TILE0) {
    tileParams->s.translate = offset * 0.5f;
    tileParams->t.translate = offset * 0.0f;

    tileParams->s.translate = fm_fmodf(tileParams->s.translate, 32.0f);
    tileParams->t.translate = fm_fmodf(tileParams->t.translate, 32.0f);
  }
}

void map_render_callback(void* data, struct render_batch* batch) {
    struct map* map = (struct map*)data;

    T3DMat4FP *mtxfp = render_batch_get_transformfp(batch);

    if (!mtxfp)
    {
        return;
    }

    mat4x4 mtx;
    transformToMatrix(&map->transform, mtx);
    mtx[3][0] *= SCENE_SCALE;
    mtx[3][1] *= SCENE_SCALE;
    mtx[3][2] *= SCENE_SCALE;
    t3d_mat4_to_fixed_3x4(mtxfp, (T3DMat4 *)mtx);

    t3d_matrix_push(mtxfp);

    t3d_model_draw_custom(map->model->t3d_model, (T3DModelDrawConf){
                                          .userData = &map->tileOffset,
                                          .tileCb = tile_scroll,
                                      });

    t3d_matrix_pop(1);
}

void map_custom_render(void* data, struct render_batch* batch) {

    render_batch_add_callback(batch, NULL, map_render_callback, data);
}

void map_update(struct map* map) {
    map->tileOffset += 0.5f;
    // Vector3 rotAxis = {0, 1, 0};
    // quatRotateAxisEuler(&map->transform.rotation, &rotAxis, 0.01f, &map->transform.rotation);
}


void map_init(struct map* map) {
    transformInitIdentity(&map->transform);
    renderable_init(&map->renderable, &map->transform, "rom:/models/map/map.t3dm");


    map->transform.position = (Vector3){0,0,0};
    map->transform.scale = (Vector3){5.0f, 5.0f, 5.0f};
    map->tileOffset = 0.0f;

    map->model = model_cache_load("rom:/models/map/map.t3dm");

    render_scene_add_renderable(&map->renderable, 1.0f);
    // render_scene_add(&map->transform.position, 80.0f, map_custom_render, map);

    // update_add(map, (update_callback)map_update, UPDATE_PRIORITY_WORLD, UPDATE_LAYER_WORLD);

}

void map_destroy(struct map* map) {
    renderable_destroy(&map->renderable);
    render_scene_remove(&map->renderable);
}