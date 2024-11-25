#include <libdragon.h>
#include <t3d/t3d.h>
#include <t3d/t3dmath.h>
#include <t3d/t3dmodel.h>
#include <t3d/t3dskeleton.h>
#include <t3d/t3danim.h>
#include <t3d/t3ddebug.h>

#include "time/time.h"
#include "render/frame_alloc.h"
#include "render/render_scene.h"
#include "collision/collision_scene.h"
#include "collision/mesh_collider.h"

#include "player/player.h"
#include "map/map.h"
#include "objects/box/box.h"
#include "objects/cone/cone.h"
#include "objects/cylinder/cylinder.h"
#include "objects/platform/platform.h"
#include "effects/fire.h"
#include "skybox/skybox_flat.h"
#include "math/transform.h"
#include "math/vector2.h"
#include "math/vector3.h"

#include "render/camera.h"
#include "scene/camera_controller.h"

#include "resource/model_cache.h"
#include "resource/mesh_collider.h"
#include "render/renderable.h"

#include "collectables/collectable.h"

#include "render/defs.h"

#include <malloc.h>

#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>

volatile static int frame_happened = 0;

static struct frame_memory_pool frame_memory_pools[2];
static uint8_t next_frame_memoy_pool;

uint8_t colorAmbient[4] = {0xAA, 0xAA, 0xAA, 0xFF};
uint8_t colorDir[4] = {0xAA, 0xAA, 0xAA, 0xFF};
Vector3 lightDirVec = {{1.0f, 1.0f, -1.0f}};

#define NUM_BOXES 5
#define NUM_COINS 5

struct player player;
struct map map;
struct box boxes[NUM_BOXES];
struct collectable coins[NUM_COINS];
struct cone cone;
struct cylinder cylinder;
struct platform plat;
struct fire fire;
struct skybox_flat skybox_flat;
struct mesh_collider test_mesh_collider;
int render_collision = 0;

struct camera camera;
struct camera_controller camera_controller;

float waiting_sec = 0.0f;

struct player_definition playerDef = {
    (Vector3){{95, 0.0f, -127}},
    (Vector2){{1, 0}}
};

struct box_definition box_def = {
    (Vector3){{85, 5, -127}}
};

struct cone_definition cone_def = {
    (Vector3){{120, -2, -141}}
};

struct collectable_definition collectableDef = {
    (Vector3){{86, 0, -144}},
    (Vector2){{1, 0}},
    COLLECTABLE_TYPE_COIN,
    0
};

struct cylinder_definition cyl_def = {
    (Vector3){{60, 0, -95}}
};

struct platform_definition plat_def = {
    (Vector3){{61, 12, -36}}
};

void on_vi_interrupt()
{
    frame_happened = 1;
}

void setup()
{
    // TODO: load initial world state, for now load meshes and animations manually
    render_scene_reset();
    update_reset();
    collision_scene_reset();
    collectable_assets_load();
    camera_init(&camera, 70.0f, 2.0f, 110.0f);
    skybox_flat_init(&skybox_flat);
    map_init(&map);
    for(int i = 0; i < NUM_BOXES; i++){
        box_init(&boxes[i], &box_def);
        box_def.position.y += 10;
    }
    for(int i = 0; i < NUM_COINS; i++){
        collectable_init(&coins[i], &collectableDef);
        collectableDef.position.x += 5;
    }

    cone_init(&cone, &cone_def);
    cylinder_init(&cylinder, &cyl_def);

    platform_init(&plat, &plat_def);
    // fire_init(&fire);

    player_init(&player, &playerDef, &camera.transform);

    camera_controller_init(&camera_controller, &camera, &player);
    
    //TODO: write scene file format & collision file format
    // mesh_collider_load_test(&test_mesh_collider);
    mesh_collider_load(&test_mesh_collider, "rom:/maps/bob_omb_battlefield/bob_map.cmsh");
    collision_scene_use_static_collision(&test_mesh_collider);
    
}

void render3d()
{
    #ifdef DEBUG_COLLIDERS_RAYLIB
    if(render_collision){

        Camera3D raylib_cam = {0};
        raylib_cam.position = (Raylib_Vector3){camera.transform.position.x * SCENE_SCALE, camera.transform.position.y * SCENE_SCALE, camera.transform.position.z * SCENE_SCALE};
        raylib_cam.target = (Raylib_Vector3){
            camera_controller.target.x * SCENE_SCALE, 
            (camera_controller.target.y + CAMERA_FOLLOW_HEIGHT) * SCENE_SCALE, 
            camera_controller.target.z * SCENE_SCALE};
        raylib_cam.up = (Raylib_Vector3){0.0f, 1.0f, 0.0f};
        raylib_cam.fovy = camera.fov;
        raylib_cam.projection = CAMERA_PERSPECTIVE;
        rlSetClipPlanes(camera.near * SCENE_SCALE, camera.far * SCENE_SCALE);
        
        BeginDrawing();
        BeginMode3D(raylib_cam);
        ClearBackground(BLACK);
        collision_scene_render_debug_raylib();
        EndMode3D();

        EndDrawing();
    }
    else{
    #endif
        // ======== Draw (3D) ======== //
        t3d_frame_start();

        // TODO: maybe move this into scene structure later so levels can have their own fog settings
        struct render_fog_params fog = {
            .enabled = true,
            .start = 30.0f * SCENE_SCALE,
            .end = 150.0f * SCENE_SCALE,
            .color = RGBA32(230, 230, 230, 0xFF)};

        t3d_screen_clear_color(fog.enabled ? fog.color : RGBA32(0, 0, 0, 0xFF));
        t3d_screen_clear_depth();

        t3d_light_set_ambient(colorAmbient);
        t3d_light_set_directional(0, colorDir, (T3DVec3*)&lightDirVec);
        t3d_light_set_count(1);

        struct frame_memory_pool *pool = &frame_memory_pools[next_frame_memoy_pool];
        frame_pool_reset(pool);

        T3DViewport *viewport = frame_malloc(pool, sizeof(T3DViewport));
        *viewport = t3d_viewport_create();

        rdpq_set_mode_standard();

        camera_apply(&camera, viewport, &camera_controller);

        render_scene_render(&camera, viewport, &frame_memory_pools[next_frame_memoy_pool], &fog);
    #ifdef DEBUG_COLLIDERS_RAYLIB
    }
    #endif
    
}

void render()
{
    // ======== Draw (3D) ======== //
    render3d();

    // ======== Draw (UI) ======== //
    //TODO: Pack UI in its own function and register UI update callbacks
    float posX = 16;
    float posY = 24;
    float fps = display_get_fps();

    struct mallinfo mall_inf = mallinfo();
    int ram_used = mall_inf.uordblks + ((uint32_t)HEAP_START_ADDR) - 0x80000000 - 0x10000;
    struct collision_scene *collision_scene = collision_scene_get();
    float aabbtree_objects_mem = (sizeof(AABBTree) + (sizeof(AABBTreeNode)) * collision_scene->object_aabbtree._nodeCount) / 1024.0f;
    float aabb_tree_mesh_mem = (sizeof(AABBTree) + (sizeof(AABBTreeNode)) * collision_scene->mesh_collider->aabbtree._nodeCount) / 1024.0f;

    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY, "[A] Attack: %d", player.is_attacking);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 10, "[B] Jump: %d", player.is_jumping);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 30, "fps: %.1f, dT: %lu", fps, TICKS_TO_MS(deltatime_ticks));
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 40, "mem: %d", ram_used);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 50, "BVH dyn, n: %d, mem: %.2fKB", collision_scene->object_aabbtree._nodeCount, aabbtree_objects_mem);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 60, "BVH mesh, n: %d, mem: %.2fKB", collision_scene->mesh_collider->aabbtree._nodeCount, aabb_tree_mesh_mem);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 70, "grounded? %d", player.is_on_ground);

    posY = 200;
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY, "Pos: %.2f, %.2f, %.2f", player.transform.position.x, player.transform.position.y, player.transform.position.z);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 20, "Vel: %.2f, %.2f, %.2f", player.physics.velocity.x, player.physics.velocity.y, player.physics.velocity.z);
    // rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 20, "Grounded: %d",  player.collision.is_grounded);
}

int main()
{
    debug_init_isviewer();
    debug_init_usblog();
    dfs_init(DFS_DEFAULT_LOCATION);

    #ifdef DEBUG_COLLIDERS_RAYLIB
    InitWindow(320, 240, "raylib test");
    SetTargetFPS(60);
    #endif

    display_init(RESOLUTION_320x240, DEPTH_16_BPP, 3, GAMMA_NONE, FILTERS_RESAMPLE_ANTIALIAS);
    display_set_fps_limit(60);

    rdpq_init();
    joypad_init();

    t3d_init((T3DInitParams){});
    rdpq_text_register_font(FONT_BUILTIN_DEBUG_MONO, rdpq_font_load_builtin(FONT_BUILTIN_DEBUG_MONO));
    
    setup();
    

    register_VI_handler(on_vi_interrupt);

    t3d_vec3_norm((T3DVec3*)&lightDirVec);
    const int64_t l_fixed_dt_ticks = TICKS_FROM_US(SEC_TO_USEC(FIXED_DELTATIME));

    debugf("Completed Initialization!\n");

    // ======== GAME LOOP ======== //
    for (;;)
    {
        waiting_sec = 0;
        uint64_t wait_ticks = TICKS_READ();
        // ======== Wait for the next frame ======== //
        while (!frame_happened)
        {

            // TODO: do something useful while waiting for the next frame
        }
        frame_happened = 0;
        wait_ticks = TICKS_READ() - wait_ticks;

        waiting_sec = (float)TICKS_TO_MS(wait_ticks) / 1000.0f;

        // ======== Update the Time ======== //

        update_time();
        accumulator_ticks += deltatime_ticks;

        // ======== Update Joypad ======== //
        joypad_poll();

        if(joypad_get_buttons_pressed(0).start){
            render_collision = !render_collision;
            // render_collision ? update_pause_layers(UPDATE_LAYER_WORLD) : update_unpause_layers(UPDATE_LAYER_WORLD);
        }

        // ======== Run the Physics and fixed Update Callbacks in a fixed Deltatime Loop ======== //
        while (accumulator_ticks >= l_fixed_dt_ticks)
        {
            fixed_update_dispatch();
            if (update_has_layer(UPDATE_LAYER_WORLD))
            {
                collision_scene_step();
            }
            accumulator_ticks -= l_fixed_dt_ticks;
        }

        // ======== Run the Update Callbacks ======== //
        update_dispatch();

        // ======== Render the Game ======== //

        surface_t* fb = display_get();
        rdpq_attach(fb, display_get_zbuf());
        render();
        rdpq_detach_wait();
        display_show(fb);
        
    }

    t3d_destroy();
    return 0;
}
