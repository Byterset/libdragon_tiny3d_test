#include "scene_loader.h"

#include <malloc.h>
#include <string.h>
#include <libdragon.h>
#include "../resource/mesh_collider.h"
#include "../resource/model_cache.h"
#include "../render/render_scene.h"
#include "../time/time.h"
// // #include "../cutscene/cutscene_runner.h"
// // #include "../cutscene/evaluation_context.h"
// // #include "../cutscene/expression_evaluate.h"

// // #include "../npc/npc.h"

// // #include "../objects/collectable.h"
#include "../objects/box/box.h"
// // #include "../objects/ground_torch.h"
// // #include "../objects/training_dummy.h"
// // #include "../objects/treasure_chest.h"

#include "../collision/collision_scene.h"

static struct entity_definition scene_entity_definitions[] = {
    // ENTITY_DEFINITION(box),
    // ENTITY_DEFINITION(collectable),
    // ENTITY_DEFINITION(crate),
    // ENTITY_DEFINITION(ground_torch),
    // ENTITY_DEFINITION(npc),
    // ENTITY_DEFINITION(training_dummy),
    // ENTITY_DEFINITION(treasure_chest),
};

// SCNE
#define EXPECTED_HEADER 0x53434e45

struct entity_definition *scene_find_def(const char *name)
{
    for (int i = 0; i < sizeof(scene_entity_definitions) / sizeof(*scene_entity_definitions); i += 1)
    {
        struct entity_definition *def = &scene_entity_definitions[i];

        if (strcmp(name, def->name) == 0)
        {
            return def;
        }
    }

    return NULL;
}

struct type_location
{
    uint8_t type;
    uint8_t offset;
};

enum type_location_types
{
    TYPE_LOCATION_STRING,
};

void scene_apply_types(void *definition, char *string_table, struct type_location *type_locations, int type_location_count)
{
    for (int i = 0; i < type_location_count; i += 1)
    {
        switch (type_locations[i].type)
        {
        case TYPE_LOCATION_STRING:
        {
            char **entry_location = (char **)((char *)definition + type_locations[i].offset);
            *entry_location += (int)string_table;
            break;
        }
        }
    }
}

//EXPR
#define EXPECTED_CONDITION_HEADER 0x45585052

// bool scene_load_check_condition(FILE *file)
// {
//     struct expression expression;
//     int header;
//     fread(&header, 1, 4, file);
//     assert(header == EXPECTED_CONDITION_HEADER);
//     uint16_t byte_size;
//     fread(&byte_size, 1, 2, file);
//     char expression_program[byte_size];
//     expression.expression_program = expression_program;
//     fread(expression.expression_program, 1, byte_size, file);

//     struct evaluation_context eval_context;
//     evaluation_context_init(&eval_context, 0);

//     expression_evaluate(&eval_context, &expression);

//     int result = evaluation_context_pop(&eval_context);

//     evaluation_context_destroy(&eval_context);

//     return result != 0;
// }

void scene_load_entity(struct scene *scene, struct entity_data *entity_data, FILE *file)
{
    uint8_t name_len;
    fread(&name_len, 1, 1, file);
    char name[name_len + 1];
    fread(name, 1, name_len, file);
    name[name_len] = '\0';

    struct entity_definition *def = scene_find_def(name);

    assert(def);

    fread(&entity_data->entity_count, 2, 1, file);
    uint16_t definition_size;
    fread(&definition_size, 2, 1, file);
    assert(definition_size == def->definition_size);

    uint8_t type_location_count;

    fread(&type_location_count, 1, 1, file);

    struct type_location type_locations[type_location_count];
    fread(type_locations, sizeof(struct type_location), type_location_count, file);

    char *entity = malloc(def->entity_size * entity_data->entity_count);
    char entity_def_data[definition_size * entity_data->entity_count];
    char *entity_def = entity_def_data;

    entity_data->definition = def;
    entity_data->entities = entity;

    fread(entity_def_data, definition_size, entity_data->entity_count, file);

    int final_count = 0;

    // for (int entity_index = 0; entity_index < entity_data->entity_count; entity_index += 1)
    // {
    //     if (scene_load_check_condition(file))
    //     {
    //         scene_apply_types(entity_def, scene->string_table, type_locations, type_location_count);
    //         def->init(entity, entity_def);
    //         entity += def->entity_size;
    //         final_count += 1;
    //     }

    //     entity_def += def->definition_size;
    // }

    entity_data->entity_count = final_count;
}

void scene_destroy_entity(struct entity_data *entity_data)
{
    char *entity = entity_data->entities;

    for (int entity_index = 0; entity_index < entity_data->entity_count; entity_index += 1)
    {
        entity_data->definition->destroy(entity);
        entity += entity_data->definition->entity_size;
    }

    free(entity_data->entities);
}

/// @brief Load a scene from a file
/// @param filename path to the scene file
/// @return the loaded scene
struct scene *scene_load(const char *filename)
{
    FILE *file = asset_fopen(filename, NULL);

    // check if the file is a valid Scene File
    int header;
    fread(&header, 1, 4, file);
    assert(header == EXPECTED_HEADER);

    struct scene *scene = malloc(sizeof(struct scene));

    // load the player definition
    struct player_definition player_def;
    player_def.location = gZeroVec;
    player_def.rotation = gRight2;

    //load the number of locations
    uint8_t location_count;
    fread(&location_count, 1, 1, file);

    bool found_entry = false;

    // Load all Locations in the Scene file and find the player starting location
    for (int i = 0; i < location_count; i += 1)
    {
        uint8_t name_length;
        fread(&name_length, 1, 1, file);
        char name[name_length + 1];
        fread(name, name_length, 1, file);
        name[name_length] = '\0';

        // uint8_t on_enter_length;
        // fread(&on_enter_length, 1, 1, file);
        // char on_enter[on_enter_length + 1];
        // fread(on_enter, on_enter_length, 1, file);
        // on_enter[on_enter_length] = '\0';

        Vector3 pos;
        Vector2 rot;

        fread(&pos, sizeof(Vector3), 1, file);
        fread(&rot, sizeof(Vector2), 1, file);

        if (found_entry)
        {
            continue;
        }

        if (strcmp(name, "default") == 0)
        {
            player_def.location = pos;
            player_def.rotation = rot;
        }

        if (strcmp(name, scene_get_next_entry()) == 0)
        {
            player_def.location = pos;
            player_def.rotation = rot;
            found_entry = true;

            // if (on_enter_length)
            // {
            //     starting_cutscene = cutscene_load(on_enter);
            // }
        }
    }

    camera_init(&scene->camera, 70.0f, 1.0f, 150.0f);
    player_init(&scene->player, &player_def, &scene->camera.transform);
    camera_controller_init(&scene->camera_controller, &scene->camera, &scene->player);

    // pause_menu_init(&scene->pause_menu);
    // hud_init(&scene->hud, &scene->player);

    // Read the number of static entities
    uint16_t static_count;
    fread(&static_count, 2, 1, file);

    // Allocate memory for the static entities
    scene->static_entities = malloc(sizeof(struct static_entity) * static_count);
    scene->static_entity_count = static_count;

    // for (int i = 0; i < static_count; ++i)
    // {
    //     uint8_t str_len;
    //     fread(&str_len, 1, 1, file);

    //     struct static_entity *entity = &scene->static_entities[i];
    //     model_cache_load(&scene->static_entities[i].model, file);
    // }

    mesh_collider_load(&scene->mesh_collider, filename, 1, NULL);
    collision_scene_use_static_collision(&scene->mesh_collider);

    uint16_t strings_length;
    fread(&strings_length, 2, 1, file);

    scene->string_table = malloc(strings_length);
    fread(scene->string_table, strings_length, 1, file);

    fread(&scene->entity_data_count, 2, 1, file);

    scene->entity_data = malloc(sizeof(struct entity_data) * scene->entity_data_count);

    for (int i = 0; i < scene->entity_data_count; i += 1)
    {
        scene_load_entity(scene, &scene->entity_data[i], file);
    }

    fread(&scene->loading_zone_count, 2, 1, file);

    scene->loading_zones = malloc(sizeof(struct loading_zone) * scene->loading_zone_count);
    fread(scene->loading_zones, sizeof(struct loading_zone), scene->loading_zone_count, file);

    for (int i = 0; i < scene->loading_zone_count; i += 1)
    {
        scene->loading_zones[i].scene_name += (int)scene->string_table;
    }

    fclose(file);

    render_scene_add_callback(NULL, 0.0f, scene_render, scene);
    update_add(scene, scene_update, UPDATE_PRIORITY_CAMERA, UPDATE_LAYER_WORLD);

    // if (starting_cutscene)
    // {
    //     cutscene_runner_run(starting_cutscene, cutscene_runner_free_on_finish(), NULL);
    // }

    return scene;
}

void scene_release(struct scene *scene)
{
    if (!scene)
    {
        return;
    }

    for (int i = 0; i < scene->static_entity_count; ++i)
    {
        struct static_entity *entity = &scene->static_entities[i];
        model_cache_release(&entity->model);
    }
    free(scene->static_entities);

    render_scene_remove(scene);
    update_remove(scene);

    // pause_menu_destroy(&scene->pause_menu);
    // hud_destroy(&scene->hud);
    player_destroy(&scene->player);
    camera_controller_destroy(&scene->camera_controller);

    // inventory_destroy();

    collision_scene_remove_static_collision();
    mesh_collider_release(&scene->mesh_collider);

    for (int i = 0; i < scene->entity_data_count; i += 1)
    {
        scene_destroy_entity(&scene->entity_data[i]);
    }
    free(scene->entity_data);

    free(scene->string_table);
    free(scene->loading_zones);

    free(scene);
}