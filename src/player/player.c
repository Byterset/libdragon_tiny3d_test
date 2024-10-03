#include "player.h"

#include <libdragon.h>

#include "../render/render_scene.h"
#include "../collision/collision_scene.h"
#include "../collision/shapes/capsule.h"
#include "../collision/shapes/cylinder.h"
#include "../collision/shapes/sphere.h"
#include "../collision/shapes/box.h"
#include "../time/time.h"
#include "../entity/entity_id.h"
#include "../render/defs.h"

#define PLAYER_MAX_SPEED    4.2f

static struct Vector2 player_max_rotation;
rspq_block_t* player_dpl;

// static struct dynamic_object_type player_collision = {
//     .minkowsi_sum = capsule_minkowski_sum,
//     .bounding_box = capsule_bounding_box,
//     .data = {
//         .capsule = {
//             .radius = 2.0f,
//             .inner_half_height = 0.1f,
//         }
//     }
// };

static struct dynamic_object_type player_collision = {
    .minkowsi_sum = box_minkowski_sum,
    .bounding_box = box_bounding_box,
    .data = {
        // .sphere = {
        //     .radius = 2.0f,
        // }
        .box = {
            .half_size = {2.0f, 1.0f, 2.0f}
        }
    }
};

void player_get_move_basis(struct Transform* transform, struct Vector3* forward, struct Vector3* right) {
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


void player_update(struct player* player) {
    struct Vector3 right;
    struct Vector3 forward;
    float animBlend = 0.4f;

    joypad_inputs_t input = joypad_get_inputs(0);
    joypad_buttons_t pressed = joypad_get_buttons_pressed(0);

    player_get_move_basis(player->camera_transform, &forward, &right);

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
        player->collision.velocity.y = 5.0f;
    }

    // Update the animation and modify the skeleton, this will however NOT recalculate the matrices
    t3d_anim_update(&player->animations.idle, frametime_sec);
    // t3d_anim_set_speed(&animWalk, animBlend + 0.15f);
    t3d_anim_update(&player->animations.walk, frametime_sec);

    if(player->is_attacking) {
      t3d_anim_update(&player->animations.attack, frametime_sec); // attack animation now overrides the idle one
      animBlend = 0.3f;
      if(!player->animations.attack.isPlaying)player->is_attacking = false;
    }
    if(player->is_jumping) {
      t3d_anim_update(&player->animations.jump, frametime_sec); // attack animation now overrides the idle one
      animBlend = 0.1f;
      if(!player->animations.jump.isPlaying)player->is_jumping = false;
    }

    // We now blend the walk animation with the idle/attack one
    t3d_skeleton_blend(&player->renderable.model->skeleton, &player->renderable.model->skeleton, &player->skelBlend, animBlend);

    t3d_skeleton_update(&player->renderable.model->skeleton);



    struct Vector2 direction;

    direction.x = input.stick_x * (1.0f / 80.0f);
    direction.y = -input.stick_y * (1.0f / 80.0f);

    float magSqrd = vector2MagSqr(&direction);

    if (magSqrd > 1.0f) {
        vector2Scale(&direction, 1.0f / sqrtf(magSqrd), &direction);
    }

    struct Vector3 directionWorld;
    vector3Scale(&right, &directionWorld, direction.x);
    vector3AddScaled(&directionWorld, &forward, direction.y, &directionWorld);

    float prev_y = player->collision.velocity.y;
    vector3Scale(&directionWorld, &player->collision.velocity, PLAYER_MOVE_SPEED);
    player->collision.velocity.y = prev_y;

    player->transform.position.x += directionWorld.x * frametime_sec * PLAYER_MOVE_SPEED;
    player->transform.position.z += directionWorld.z * frametime_sec * PLAYER_MOVE_SPEED;

    if (magSqrd > 0.01f) {
        struct Vector2 directionUnit;

        directionUnit.x = directionWorld.x;
        directionUnit.y = directionWorld.z;

        vector2Normalize(&directionUnit, &directionUnit);

        float tmp = directionUnit.x;
        directionUnit.x = directionUnit.y;
        directionUnit.y = tmp;

        vector2ComplexFromAngle(frametime_sec * PLAYER_TURN_SPEED, &player_max_rotation);

        vector2RotateTowards(&player->look_direction, &directionUnit, &player_max_rotation, &player->look_direction);
    }

    quatAxisComplex(&gUp, &player->look_direction, &player->transform.rotation);

    // // use blend based on speed for smooth transitions
    // animBlend = currSpeed / 0.51f;
    // if(animBlend > 1.0f)animBlend = 1.0f;

    // move player...
    // player->transform.position.x += moveDir.x * currSpeed;
    // player->transform.position.z += moveDir.z * currSpeed;
    // ...and limit position inside the box
    const float BOX_SIZE = 60.0f;
    if(player->transform.position.x < -BOX_SIZE)player->transform.position.x = -BOX_SIZE;
    if(player->transform.position.x >  BOX_SIZE)player->transform.position.x =  BOX_SIZE;
    if(player->transform.position.z < -BOX_SIZE)player->transform.position.z = -BOX_SIZE;
    if(player->transform.position.z >  BOX_SIZE)player->transform.position.z =  BOX_SIZE;
    if(player->transform.position.y <= 0){
        player->transform.position.y = 0;
        player->collision.velocity.y = player->collision.velocity.y <= 0 ? 0 : player->collision.velocity.y;    
    }
    if(player->collision.velocity.y <= -100.0f){
        player->collision.velocity.y = -100.0f;
    }

    // struct contact* contact = player->collision.active_contacts;

    // while (contact) {
    //     struct collectable* collectable = collectable_get(contact->other_object);

    //     if (collectable) {
    //         collectable_collected(collectable);
    //     }

    //     contact = contact->next;
    // }
}



void player_init(struct player* player, struct player_definition* definition, struct Transform* camera_transform) {
    entity_id entity_id = entity_id_new();

    transformInitIdentity(&player->transform);
    
    renderable_init(&player->renderable, &player->transform, "rom:/models/snake/snake.t3dm");

    assert(player->renderable.model->has_skeleton && &player->renderable.model->t3d_model != NULL);

    
    player->skelBlend = t3d_skeleton_clone(&player->renderable.model->skeleton, false);
    player->transform.scale = (struct Vector3){1, 1, 1};
    player->camera_transform = camera_transform;

    player->transform.position = definition->location;

    render_scene_add_renderable(&player->renderable, 2.0f);


    
    update_add(player, (update_callback)player_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_PLAYER);

    player->look_direction = definition->rotation;

    

    dynamic_object_init(
        entity_id,
        &player->collision,
        &player_collision,
        COLLISION_LAYER_TANGIBLE | COLLISION_LAYER_DAMAGE_PLAYER,
        &player->transform.position,
        &player->look_direction,
        20.0f
    );

    player->collision.collision_group = COLLISION_GROUP_PLAYER;

    player->collision.has_gravity = true;
    

    player->collision.center.y = player_collision.data.box.half_size.y;
    // player->collision.center.y = player_collision.data.sphere.radius;

    collision_scene_add(&player->collision);

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
    collision_scene_remove(&player->collision);
    t3d_anim_destroy(&player->animations.idle);
    t3d_anim_destroy(&player->animations.walk);
    t3d_anim_destroy(&player->animations.attack);
    t3d_anim_destroy(&player->animations.jump);
}