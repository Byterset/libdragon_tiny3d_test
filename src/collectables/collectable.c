#include "collectable.h"

#include "../collision/collision_scene.h"
#include "../collision/shapes/sphere.h"
#include "../render/render_scene.h"
#include "../resource/material_cache.h"
#include "../time/time.h"
#include "../util/hash_map.h"

#define COLLECTABLE_RADIUS  0.75f

static struct physics_object_collision_data collectable_collision = {
    .gjk_support_function = sphere_support_function,
    .bounding_box_calculator = sphere_bounding_box,
    .shape_data = {
        .sphere = {
            .radius = COLLECTABLE_RADIUS,
        }
    },
    .bounce = 0.2f,
    .friction = 0.25f,
    .shape_type = COLLISION_SHAPE_SPHERE,
};

static struct hash_map collectable_hash_map;

struct collectable_information {
    char* mesh_filename;
};

static struct collectable_information collectable_information[] = {
    [COLLECTABLE_TYPE_COIN] = {
        .mesh_filename = "rom:/models/box/box.t3dm",
    },

};

void collectable_assets_load() {
    hash_map_init(&collectable_hash_map, 8);
}

void collectable_init(struct collectable* collectable, struct collectable_definition* definition) {
    collectable->collectable_type = definition->collectable_type;
    collectable->collectable_sub_type = definition->collectable_sub_type;
    collectable->transform.position = definition->position;
    collectable->transform.rotation = definition->rotation;
    
    physics_object_init(
        entity_id_new(), 
        &collectable->physics, 
        &collectable_collision, 
        COLLISION_LAYER_COLLECTABLES,
        &collectable->transform.position, 
        NULL,
        1.0f
    );
    collectable->physics.collision_group = COLLISION_GROUP_COLLECTABLE;
    collectable->physics.is_fixed = 1;
    collectable->physics.is_trigger = 1;
    collectable->physics.has_gravity = 0;

    struct collectable_information* type = &collectable_information[definition->collectable_type];

    collision_scene_add(&collectable->physics);
    renderable_single_axis_init(&collectable->renderable, &collectable->transform, type->mesh_filename);
    render_scene_add_renderable_single_axis(&collectable->renderable, 0.2f);
    
    hash_map_set(&collectable_hash_map, collectable->physics.entity_id, collectable);
}

void collectable_collected(struct collectable* collectable) {
    collectable_destroy(collectable);

    if (collectable->collectable_type == COLLECTABLE_TYPE_COIN) {
        debugf("Collected coin\n");
    }
}

void collectable_destroy(struct collectable* collectable) {
    collision_scene_remove(&collectable->physics);
    render_scene_remove(&collectable->renderable);
    renderable_single_axis_destroy(&collectable->renderable);
    hash_map_delete(&collectable_hash_map, collectable->physics.entity_id);
}

struct collectable* collectable_get(entity_id id) {
    return hash_map_get(&collectable_hash_map, id);
}