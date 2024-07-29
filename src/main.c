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
#include "math/transform.h"
#include "math/vector2.h"
#include "math/vector3.h"

#include "render/camera.h"
#include "scene/camera_controller.h"

volatile static int frame_happened = 0;

static struct frame_memory_pool frame_memory_pools[2];
static uint8_t next_frame_memoy_pool;

uint8_t colorAmbient[4] = {0xAA, 0xAA, 0xAA, 0xFF};
uint8_t colorDir[4] = {0xFF, 0xAA, 0xAA, 0xFF};
T3DVec3 lightDirVec = {{1.0f, 1.0f, 1.0f}};


float currSpeed = 0.0f;
float animBlend = 0.0f;
bool isAttack = false;
bool isJump = false;

rspq_syncpoint_t syncPoint = 0;

struct player player;
struct map map;

struct Camera camera;
struct camera_controller camera_controller;

struct player_definition playerDef = {
    (struct Vector3){0, 0.15f, 0},
    (struct Vector2){1, 0}
};


void setup(){
  //TODO: load initial world state, for now load meshes and animations manually
  render_scene_reset();
  update_reset();
  collision_scene_reset();
  camera_init(&camera, 70.0f, 1.0f, 360.0f);
  // camera.transform.position = (struct Vector3){0, 45.0f, 80.0f};
  map_init(&map);
  player_init(&player, &playerDef, &camera.transform);

  camera_controller_init(&camera_controller, &camera, &player);

  //TODO: implement mesh collision new
  // mesh_collider_load(&world->mesh_collider, file);
  // collision_scene_use_static_collision(&world->mesh_collider);
}

void render3d(){
   // ======== Draw (3D) ======== //
    t3d_frame_start();

    struct frame_memory_pool *pool = &frame_memory_pools[next_frame_memoy_pool];
    frame_pool_reset(pool);

    T3DViewport *viewport = frame_malloc(pool, sizeof(T3DViewport));
    *viewport = t3d_viewport_create();
    t3d_viewport_attach(viewport);

    t3d_screen_clear_color(RGBA32(0, 180, 180, 0xFF));
    t3d_screen_clear_depth();

    // position the camera behind the player
    T3DVec3 camPos, camTarget;
    camTarget.v[0] = player.transform.position.x;
    camTarget.v[1] = player.transform.position.y;
    camTarget.v[2] = player.transform.position.z;
    camTarget.v[2] -= 20;
    camPos.v[0] = camTarget.v[0];
    camPos.v[1] = camTarget.v[1] + 45;
    camPos.v[2] = camTarget.v[2] + 65;

    // t3d_viewport_set_projection(&viewport, T3D_DEG_TO_RAD(85.0f), 10.0f, 150.0f);
    t3d_viewport_look_at(viewport, &camPos, &camTarget, &(T3DVec3){{0,1,0}});

    t3d_light_set_ambient(colorAmbient);
    t3d_light_set_directional(0, colorDir, &lightDirVec);
    t3d_light_set_count(1);

    rdpq_mode_fog(RDPQ_FOG_STANDARD);
    rdpq_set_fog_color((color_t){140, 50, 20, 0xFF});

    t3d_fog_set_enabled(true);
    t3d_fog_set_range(0.4f, 400.0f);

    render_scene_render(&camera, viewport, &frame_memory_pools[next_frame_memoy_pool]);
}

void render(surface_t* zbuffer) {
    update_render_time();

    render3d();

    // ======== Draw (UI) ======== //
    //TODO: Pack UI in its own function and register UI update callbacks
    float posX = 16;
    float posY = 24;

    rdpq_sync_pipe();
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY, "[A] Attack: %d", player.is_attacking);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 10, "[B] Jump: %d", player.is_jumping);
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY + 20, "FPS: %.6f", (render_time_step) );
    posY = 200;
    rdpq_text_printf(NULL, FONT_BUILTIN_DEBUG_MONO, posX, posY, "Pos: %.2f, %.2f, %.2f", player.transform.position.x, player.transform.position.y, player.transform.position.z);

    syncPoint = rspq_syncpoint_new();
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

  t3d_vec3_norm(&lightDirVec);


  // ======== GAME LOOP ======== //
  for(;;)
  {
    // while(!frame_happened){
    //   // TODO: do something useful while waiting for the next frame
    // }

    // ======== Update ======== //
    joypad_poll();

    if(syncPoint)rspq_syncpoint_wait(syncPoint); // wait for the RSP to process the previous frame

    surface_t *fb = display_try_get();

    if (fb)
    {
      rdpq_attach(fb, &zbuffer);

      render(&zbuffer);

      rdpq_detach_show();
    }

    if (update_has_layer(UPDATE_LAYER_WORLD))
    {
      collision_scene_collide();
    }

    update_dispatch();
  }

  t3d_destroy();
  return 0;
}

