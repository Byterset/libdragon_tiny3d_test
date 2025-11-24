#include "player.h"

#include <libdragon.h>
#include <t3d/t3d.h>

#include "../render/render_scene.h"
#include "../collision/collision_scene.h"
#include "../collision/shapes/capsule.h"
#include "../collision/shapes/cylinder.h"
#include "../collision/shapes/sphere.h"
#include "../time/time.h"
#include "../entity/entity_id.h"
#include "../render/defs.h"
#include "../collectables/collectable.h"

#define PLAYER_MAX_SPEED    50.0f
#define PLAYER_MAX_ACC       80.0f
#define PLAYER_MAX_ACC_AIR   40.0f
#define PLAYER_MAX_ANGLE_GROUND 45.0f
#define PLAYER_MAX_ANGLE_GROUND_DOT cosf(T3D_DEG_TO_RAD(PLAYER_MAX_ANGLE_GROUND))
#define PLAYER_JUMP_HEIGHT  5.2f
#define PLAYER_TURN_SPEED 20.0f

static Vector2 player_max_rotation;

static struct physics_object_collision_data player_collision = {
    CAPSULE_COLLIDER(1.0f, 0.7f),
    .friction = 0.3f,
    .bounce = 0
};

void player_handle_contacts(struct player* player){
    contact *contact = player->physics.active_contacts;
    while (contact)
    {
        struct collectable *collectable = contact->other_object ? collectable_get(contact->other_object->entity_id) : NULL;

        if (collectable)
        {
            collectable_collected(collectable);
        }

        if ((contact->other_object && contact->other_object->collision_layers & COLLISION_LAYER_TANGIBLE) || !contact->other_object)
        {
            if (contact->constraint->normal.y >= PLAYER_MAX_ANGLE_GROUND_DOT)
            {
                player->is_on_ground = true;
                vector3Add(&player->ground_normal, &contact->constraint->normal, &player->ground_normal);
            }
        }

        contact = contact->next;
    }
}

void player_get_move_basis(Transform* transform, Vector3* forward, Vector3* right) {
    quatMultVector(&transform->rotation, &gForward, forward);
    quatMultVector(&transform->rotation, &gRight, right);

    if (forward->y > 0.7f) {
        quatMultVector(&transform->rotation, &gUp, forward);
        vector3Negate(forward, forward);
    } else if (forward->y < -0.7f) {
        quatMultVector(&transform->rotation, &gUp, forward);
    }

    forward->y = 0.0f;
    right->y = 0.0f;

    vector3Normalize(forward, forward);
    vector3Normalize(right, right);
}

void player_reset_state(struct player* player){
    if (!player->physics._is_sleeping)
    {
        player->is_on_ground = false;
    }
    player->ground_normal = gZeroVec;
}



void player_fixed_update(struct player* player){

    float acc = PLAYER_MAX_ACC_AIR;
    if (player->is_on_ground)
    {
        acc = PLAYER_MAX_ACC;
        // ground normal was summed up in the handle contacts function so we need to normalize it
        vector3Normalize(&player->ground_normal, &player->ground_normal);
    }
    else
    {
        player->ground_normal = gUp;
    }
    Vector3 xAxis;
    vector3ProjectPlane(&gRight, &player->ground_normal, &xAxis);
    Vector3 zAxis;
    vector3ProjectPlane(&gForward, &player->ground_normal, &zAxis);
    vector3Normalize(&xAxis, &xAxis);
    vector3Normalize(&zAxis, &zAxis);
    float currentX = vector3Dot(&player->physics.velocity, &xAxis);
	float currentZ = vector3Dot(&player->physics.velocity, &zAxis);
    float maxSpeedChange = FIXED_DELTATIME * acc;
    float newX = mathfMoveTowards(currentX, player->desired_velocity.x, maxSpeedChange);
    float newZ = mathfMoveTowards(currentZ, player->desired_velocity.z, maxSpeedChange);

    
    vector3Scale(&xAxis, &xAxis, (newX - currentX));
    vector3Scale(&zAxis, &zAxis, (newZ - currentZ));
    vector3Add(&xAxis, &player->physics.velocity, &player->physics.velocity);
    vector3Add(&zAxis, &player->physics.velocity, &player->physics.velocity);

    player_reset_state(player);

    player->ray_down_hit = (raycast_hit){0};
    player->ray_fwd_hit = (raycast_hit){0};
    Vector3 ray_origin = player->transform.position;
    ray_origin.y += 0.5f;
    Vector3 ray_dir = (Vector3){{0.0f, -1.0f, 0.0f}};
    raycast ray_down = raycast_init(ray_origin, ray_dir, 2.0f, RAYCAST_COLLISION_SCENE_MASK_ALL, false, COLLISION_LAYER_TANGIBLE, COLLISION_LAYER_PLAYER);
    ray_origin.y += 1.5f;
    ray_dir = (Vector3){{0.0f, 0.0f, 1.0f}};
    quatMultVector(&player->transform.rotation, &ray_dir, &ray_dir);
    raycast ray_fwd = raycast_init(ray_origin, ray_dir, 5.0f, RAYCAST_COLLISION_SCENE_MASK_ALL, false, COLLISION_LAYER_TANGIBLE, COLLISION_LAYER_PLAYER); 
    
    raycast_cast(&ray_down, &player->ray_down_hit);
    raycast_cast(&ray_fwd, &player->ray_fwd_hit);

}



void player_render_callback(void* data, struct render_batch* batch) {
    struct player* player = (struct player*)data;

    T3DMat4FP *mtxfp = render_batch_get_transformfp(batch);

    if (!mtxfp)
    {
        return;
    }

    Matrix4x4 mtx;
    transformToMatrix(&player->transform, &mtx);
    
    t3d_mat4_to_fixed_3x4(mtxfp, &mtx);

    rdpq_mode_persp(true);
    t3d_state_set_drawflags(T3D_FLAG_DEPTH | T3D_FLAG_SHADED | T3D_FLAG_TEXTURED | T3D_FLAG_CULL_BACK);

    t3d_matrix_push(mtxfp);

    T3DModelDrawConf conf = {
        .userData = NULL,
        .tileCb = NULL,
        .filterCb = NULL,
        .matrices = player->renderable.model->skeleton.bufferCount == 1
                        ? player->renderable.model->skeleton.boneMatricesFP
                        : (const T3DMat4FP *)t3d_segment_placeholder(T3D_SEGMENT_SKELETON)};
    T3DModelState state = t3d_model_state_create();
    state.drawConf = &conf;


    rdpq_combiner_t comb = RDPQ_COMBINER1((0,0,0,PRIM),(0,0,0,PRIM));
    rdpq_blender_t blend = RDPQ_BLENDER((IN_RGB, IN_ALPHA, MEMORY_RGB, INV_MUX_ALPHA));
    T3DModelIter it = t3d_model_iter_create(player->renderable.model->t3d_model, T3D_CHUNK_TYPE_OBJECT);

    // Draw the player once as a solig colored silhouette, ignoring the z-buffer
    // rdpq_mode_zbuf(false, false);
    // rdpq_mode_blender(blend);
    // rdpq_set_prim_color((color_t){100, 240, 255, 255});
    // rdpq_mode_combiner(comb);
    
    // while (t3d_model_iter_next(&it))
    // {
    //     t3d_model_draw_object(it.object, conf.matrices);
    // }

    // it._idx = 0;
    // it.chunk = NULL;

    // Draw the player again with the z-buffer enabled, so that the silhouette is overdrawn by the actual model
    // Thus the silhouette will only be visible where the model is not (e.g. behind walls and objects)
    rdpq_mode_zbuf(true, true);
    while (t3d_model_iter_next(&it))
    {
        if (it.object->material)
        {
            t3d_model_draw_material(it.object->material, &state);
        }
        t3d_model_draw_object(it.object, conf.matrices);
    }

    t3d_matrix_pop(1);
}

void player_custom_render(void* data, struct render_batch* batch) {
    render_batch_add_callback(batch, NULL, player_render_callback, data);
}


void player_update(struct player* player) {
    Vector3 right;
    Vector3 forward;
    float animBlend = 0.4f;

    joypad_inputs_t input = joypad_get_inputs(0);
    joypad_buttons_t pressed = joypad_get_buttons_pressed(0);
    joypad_buttons_t held = joypad_get_buttons_held(0);

    player_handle_contacts(player);

    // Player Attack
    if (pressed.a && !player->animations.attack.isPlaying)
    {
        t3d_anim_set_playing(&player->animations.attack, true);
        t3d_anim_set_time(&player->animations.attack, 0.0f);
        player->is_attacking = true;
    }
    if (pressed.b && !player->animations.jump.isPlaying)
    {
        t3d_anim_set_playing(&player->animations.jump, false);
        t3d_anim_set_time(&player->animations.jump, 0.0f);
        player->is_jumping = true;
    }
    if (pressed.b){
        float jumpVelocity = sqrtf(-2.0f * (PHYS_GRAVITY_CONSTANT * player->physics.gravity_scalar) * PLAYER_JUMP_HEIGHT); // v = sqrt(2gh)
        player->physics.velocity.y = jumpVelocity;
    }
    if (pressed.d_down){
        struct collision_scene* cs = collision_scene_get();
        player->look_target = cs->elements[randomInRange(0, cs->objectCount)].object->position;
    }

    // Update the animation and modify the skeleton, this will however NOT recalculate the matrices
    t3d_anim_update(&player->animations.idle, deltatime_sec);
    // t3d_anim_set_speed(&animWalk, animBlend + 0.15f);
    t3d_anim_update(&player->animations.walk, deltatime_sec);

    if(player->is_attacking) {
      t3d_anim_update(&player->animations.attack, deltatime_sec); // attack animation now overrides the idle one
      animBlend = 0.3f;
      if(!player->animations.attack.isPlaying)player->is_attacking = false;
    }
    if(player->is_jumping) {
      t3d_anim_update(&player->animations.jump, deltatime_sec); // attack animation now overrides the idle one
      animBlend = 0.1f;
      if(!player->animations.jump.isPlaying)player->is_jumping = false;
    }

    // We now blend the walk animation with the idle/attack one
    t3d_skeleton_blend(&player->renderable.model->skeleton, &player->renderable.model->skeleton, &player->skelBlend, animBlend);

    // t3d_skeleton_update(&player->renderable.model->skeleton);


    // Update Player Rotation and desired Velocity based on the camera and input
    player_get_move_basis(player->camera_transform, &forward, &right);

    Vector2 direction;

    direction.x = input.stick_x * (1.0f / 80.0f);
    direction.y = -input.stick_y * (1.0f / 80.0f);

    float magSqrd = vector2MagSqr(&direction);

    if (magSqrd > 1.0f) {
        vector2Scale(&direction, 1.0f / sqrtf(magSqrd), &direction);
    }

    Vector3 directionWorld;
    vector3Scale(&right, &directionWorld, direction.x);
    vector3AddScaled(&directionWorld, &forward, direction.y, &directionWorld);

    if (magSqrd > 0.01f) {
        Vector2 directionUnit;

        directionUnit.x = directionWorld.x;
        directionUnit.y = directionWorld.z;

        vector2Normalize(&directionUnit, &directionUnit);

        float tmp = directionUnit.x;
        directionUnit.x = directionUnit.y;
        directionUnit.y = tmp;

        vector2ComplexFromAngleRad(deltatime_sec * PLAYER_TURN_SPEED, &player_max_rotation);

        vector2RotateTowards(&player->look_direction, &directionUnit, &player_max_rotation, &player->look_direction);
    }
    
    quatAxisComplex(&gUp, &player->look_direction, &player->transform.rotation);


    float max_speed = held.r ? PLAYER_MAX_SPEED * 2.0f : PLAYER_MAX_SPEED;

    player->desired_velocity = (Vector3){{directionWorld.x * max_speed, 0.0f, directionWorld.z * max_speed}};

    if (player->look_target)
    {


        Quaternion mouthForwardOffset;

        quatEulerAngles(&(Vector3){{0, 0, T3D_DEG_TO_RAD(-90)}}, &mouthForwardOffset);

        Vector3 lookDir;

        int head_index = t3d_skeleton_find_bone(&player->renderable.model->skeleton, "Mouth");
        T3DBone *head_bone = &player->renderable.model->skeleton.bones[head_index];

        // TODO: actually derive the head bones position in world space somehow - for now use the player position
        vector3FromTo(&player->transform.position, player->look_target, &lookDir);

        // Negate effect of player rotation
        Quaternion inverse_player_rot;
        quatConjugate(&player->transform.rotation, &inverse_player_rot);

        // Create look-at rotation
        Quaternion lookAtQuat;
        quatLook(&lookDir, &gUp, &lookAtQuat);
        Quaternion finalHeadRot;

        quatMultiply(&lookAtQuat, &mouthForwardOffset, &finalHeadRot);
        quatMultiply(&inverse_player_rot, &finalHeadRot, &finalHeadRot);

        // Set the bone rotation (replace original, don't multiply)
        player->renderable.model->skeleton.bones[head_index].rotation = finalHeadRot;
        player->renderable.model->skeleton.bones[head_index].hasChanged = true;
    }

    t3d_skeleton_update(&player->renderable.model->skeleton);

}



void player_init(struct player* player, struct player_definition* definition, Transform* camera_transform) {
    entity_id entity_id = entity_id_new();

    transformInitIdentity(&player->transform);
    
    renderable_init(&player->renderable, &player->transform, "rom:/models/snake/snake.t3dm");

    assert(player->renderable.model->has_skeleton && &player->renderable.model->t3d_model != NULL);

    
    player->skelBlend = t3d_skeleton_clone(&player->renderable.model->skeleton, false);
    player->transform.scale = (Vector3){{1.0f, 1.0f, 1.0f}};
    player->camera_transform = camera_transform;
    player->desired_velocity = gZeroVec;
    player->is_on_ground = false;

    player->transform.position = definition->location;
    // render_scene_add_callback(NULL, 0, render_scene_render_renderable, &player->renderable);
    render_scene_add_callback(NULL, 0.0f, player_custom_render, player);
    // render_scene_add_renderable(&player->renderable, 2.0f);


    
    update_add(player, (update_callback)player_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_PLAYER);
    fixed_update_add(player, (update_callback)player_fixed_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_PLAYER);

    player->look_direction = definition->rotation;

    physics_object_init(
        entity_id,
        &player->physics,
        &player_collision,
        COLLISION_LAYER_TANGIBLE | COLLISION_LAYER_PLAYER | COLLISION_LAYER_COLLECTABLES,
        &player->transform.position,
        &player->transform.rotation,
        (Vector3){{0,player_collision.shape_data.capsule.inner_half_height + player_collision.shape_data.capsule.radius,0}},
        70.0f
    );

    player->physics.collision_group = COLLISION_GROUP_PLAYER;
    player->physics.constraints |= CONSTRAINTS_FREEZE_ROTATION_ALL;
    player->physics.gravity_scalar = 1.5f;

    collision_scene_add(&player->physics);

    player->animations.jump = t3d_anim_create(player->renderable.model->t3d_model, "Snake_Jump");
    t3d_anim_set_looping(&player->animations.jump, false); // don't loop this animation
    t3d_anim_set_playing(&player->animations.jump, false); // start in a paused state
    t3d_anim_attach(&player->animations.jump, &player->renderable.model->skeleton);

    player->animations.attack = t3d_anim_create(player->renderable.model->t3d_model, "Snake_Attack");
    t3d_anim_set_looping(&player->animations.attack, false); // don't loop this animation
    t3d_anim_set_playing(&player->animations.attack, false); // start in a paused state
    t3d_anim_attach(&player->animations.attack, &player->renderable.model->skeleton);

    player->animations.idle = t3d_anim_create(player->renderable.model->t3d_model, "Snake_Idle");
    t3d_anim_attach(&player->animations.idle, &player->renderable.model->skeleton);

    player->animations.walk = t3d_anim_create(player->renderable.model->t3d_model, "Snake_Walk");
    t3d_anim_attach(&player->animations.walk, &player->skelBlend);

}

void player_destroy(struct player* player) {
    renderable_destroy(&player->renderable);

    render_scene_remove(&player->renderable);
    update_remove(player);
    collision_scene_remove(&player->physics);
    t3d_anim_destroy(&player->animations.idle);
    t3d_anim_destroy(&player->animations.walk);
    t3d_anim_destroy(&player->animations.attack);
    t3d_anim_destroy(&player->animations.jump);
}