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

#include "player/player.h"
#include "map/map.h"
#include "objects/box/box.h"
#include "effects/fire.h"
#include "math/transform.h"
#include "math/vector2.h"
#include "math/vector3.h"

#include "render/camera.h"
#include "scene/camera_controller.h"

#include "resource/model_cache.h"
#include "render/renderable.h"

#include <malloc.h>

volatile static int frame_happened = 0;

static struct frame_memory_pool frame_memory_pools[2];
static uint8_t next_frame_memoy_pool;

uint8_t colorAmbient[4] = {0xAA, 0xAA, 0xAA, 0xFF};
uint8_t colorDir[4] = {0xFF, 0xAA, 0xAA, 0xFF};
T3DVec3 lightDirVec = {{1.0f, 1.0f, 1.0f}};

struct player player;
struct map map;
struct box box;
struct fire fire;

struct camera camera;
struct camera_controller camera_controller;


float waiting_sec = 0.0f;

struct player_definition playerDef = {
    (struct Vector3){0, 0.15f, 0},
    (struct Vector2){1, 0}
};

void on_vi_interrupt() {
    frame_happened = 1;
}

void setup(){
  //TODO: load initial world state, for now load meshes and animations manually
  render_scene_reset();
  update_reset();
  collision_scene_reset();
  camera_init(&camera, 70.0f, 1.0f, 100.0f);
  map_init(&map);
  box_init(&box, (struct Vector3){0, 1.4f, 0});
  fire_init(&fire);

  player_init(&player, &playerDef, &camera.transform);
  

  camera_controller_init(&camera_controller, &camera, &player);

  //TODO: implement mesh collision new
  // mesh_collider_load(&world->mesh_collider, file);
  // collision_scene_use_static_collision(&world->mesh_collider);
}

void render3d(){
   // ======== Draw (3D) ======== //
    t3d_frame_start();

    t3d_screen_clear_color(RGBA32(0, 0, 0, 0xFF));
    t3d_screen_clear_depth();

    t3d_light_set_ambient(colorAmbient);
    t3d_light_set_directional(0, colorDir, &lightDirVec);
    t3d_light_set_count(1);

    struct frame_memory_pool *pool = &frame_memory_pools[next_frame_memoy_pool];
    frame_pool_reset(pool);

    T3DViewport *viewport = frame_malloc(pool, sizeof(T3DViewport));
    *viewport = t3d_viewport_create();

  rdpq_set_mode_standard();
    // rdpq_mode_fog(RDPQ_FOG_STANDARD);
    // rdpq_set_fog_color((color_t){255, 255, 255, 0xFF});
  

    // t3d_fog_set_enabled(true);
    // t3d_fog_set_range(100.0f, 2000.0f);

    render_scene_render(&camera, viewport, &frame_memory_pools[next_frame_memoy_pool]);
}

void render(surface_t* zbuffer) {

    render3d();

    // ======== Draw (UI) ======== //
    //TODO: Pack UI in its own function and register UI update callbacks
    float posX = 16;
    float posY = 24;
    float fps = display_get_fps();
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY, "[A] Attack: %d", player.is_attacking);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 10, "[B] Jump: %d", player.is_jumping);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 30, "fps: %.1f", fps);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 40, "idle time: %.5f", waiting_sec);
    
    posY = 200;
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY, "Pos: %.2f, %.2f, %.2f", player.transform.position.x, player.transform.position.y, player.transform.position.z);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 10, "Vel: %.2f, %.2f, %.2f", player.collision.velocity.x, player.collision.velocity.y, player.collision.velocity.z);
}

int main()
{
  debug_init_isviewer();
  debug_init_usblog();

  dfs_init(DFS_DEFAULT_LOCATION);

  display_init(RESOLUTION_320x240, DEPTH_16_BPP, 3, GAMMA_NONE, FILTERS_RESAMPLE_ANTIALIAS);
  surface_t zbuffer = surface_alloc(FMT_RGBA16, display_get_width(), display_get_height());

  rdpq_init();
  joypad_init();

  t3d_init((T3DInitParams){});
  rdpq_text_register_font(FONT_BUILTIN_DEBUG_MONO, rdpq_font_load_builtin(FONT_BUILTIN_DEBUG_MONO));

  setup();

  register_VI_handler(on_vi_interrupt);

  t3d_vec3_norm(&lightDirVec);
  const int64_t l_dt = TICKS_FROM_US(SEC_TO_USEC(FIXED_DELTATIME));


  // ======== GAME LOOP ======== //
  for(;;)
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

    waiting_sec =  (float)TICKS_TO_MS(wait_ticks) / 1000.0f;


    // ======== Update the Time ======== //

    update_time();
    accumulator_ticks += frametime_ticks;

    // ======== Update Joypad ======== //
    joypad_poll();



    // ======== Run the Physics in a fixed Deltatime Loop ======== //
    while (accumulator_ticks >= l_dt){
      
      if (update_has_layer(UPDATE_LAYER_WORLD))
      {
        collision_scene_collide();
      }
      accumulator_ticks -= l_dt;
    }

    // ======== Run the Update Callbacks ======== //
    update_dispatch();

    // ======== Render the Game ======== //
    surface_t *fb = display_try_get();

    if (fb)
    {
      rdpq_attach(fb, &zbuffer);

      render(&zbuffer);

      rdpq_detach_show();
    }

  }

  t3d_destroy();
  return 0;
}

