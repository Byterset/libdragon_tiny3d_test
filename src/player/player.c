#include "player.h"

#include <libdragon.h>

#include "../math/vector2.h"

#include "../render/render_scene.h"
#include "../collision/collision_scene.h"
#include "../time/time.h"
#include "../entity/entity_id.h"

static struct Vector2 player_max_rotation;

static struct dynamic_object_type player_collision = {
    .minkowsi_sum = dynamic_object_capsule_minkowski_sum,
    .bounding_box = dynamic_object_capsule_bounding_box,
    .data = {
        .capsule = {
            .radius = 0.25f,
            .inner_half_height = 0.5f,
        }
    }
};

static struct dynamic_object_type player_visual_shape = {
    .minkowsi_sum = dynamic_object_cylinder_minkowski_sum,
    .bounding_box = dynamic_object_cylinder_bounding_box,
    .data = {
        .cylinder = {
            .half_height = 0.5f,
            .radius = 0.5f,
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


    joypad_inputs_t input = joypad_get_inputs(0);
    joypad_buttons_t pressed = joypad_get_buttons_pressed(0);

    player_get_move_basis(player->camera_transform, &forward, &right);


    // Update the animation and modify the skeleton, this will however NOT recalculate the matrices
    t3d_anim_update(&player->animations.idle, 0.0005f);
    // t3d_anim_set_speed(&animWalk, animBlend + 0.15f);
    t3d_anim_update(&player->animations.walk, 0.0005f);

    if(player->is_attacking) {
      t3d_anim_update(&player->animations.attack, 0.0005f); // attack animation now overrides the idle one
      if(!player->animations.attack.isPlaying)player->is_attacking = false;
    }
    if(player->is_jumping) {
      t3d_anim_update(&player->animations.jump, 0.0005f); // attack animation now overrides the idle one
      if(!player->animations.jump.isPlaying)player->is_jumping = false;
    }
    t3d_skeleton_update(&player->renderable.skeleton);

    // Player Attack
    if(pressed.a && !player->animations.attack.isPlaying) {
      t3d_anim_set_playing(&player->animations.attack, true);
      t3d_anim_set_time(&player->animations.attack, 0.0f);
      player->is_attacking = true;
    }
    if(pressed.b && !player->animations.jump.isPlaying) {
      t3d_anim_set_playing(&player->animations.jump, true);
      t3d_anim_set_time(&player->animations.jump, 0.0f);
      player->is_jumping = true;
    }

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

    // float prev_y = player->collision.velocity.y;
    // vector3Scale(&directionWorld, &player->collision.velocity, PLAYER_MAX_SPEED);
    // player->collision.velocity.y = prev_y;

    player->transform.position.x += directionWorld.x * fixed_time_step;
    player->transform.position.z += directionWorld.z * fixed_time_step;

    if (magSqrd > 0.01f) {
        struct Vector2 directionUnit;

        directionUnit.x = directionWorld.x;
        directionUnit.y = directionWorld.z;

        vector2Normalize(&directionUnit, &directionUnit);

        float tmp = directionUnit.x;
        directionUnit.x = directionUnit.y;
        directionUnit.y = tmp;

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
    const float BOX_SIZE = 140.0f;
    if(player->transform.position.x < -BOX_SIZE)player->transform.position.x = -BOX_SIZE;
    if(player->transform.position.x >  BOX_SIZE)player->transform.position.x =  BOX_SIZE;
    if(player->transform.position.z < -BOX_SIZE)player->transform.position.z = -BOX_SIZE;
    if(player->transform.position.z >  BOX_SIZE)player->transform.position.z =  BOX_SIZE;


    // struct contact* contact = player->collision.active_contacts;

    // while (contact) {
    //     struct collectable* collectable = collectable_get(contact->other_object);

    //     if (collectable) {
    //         collectable_collected(collectable);
    //     }

    //     contact = contact->next;
    // }

    if (pressed.a) {
        // player_handle_a_action(player);
    }
}

void player_init(struct player* player, struct player_definition* definition, struct Transform* camera_transform) {
    entity_id entity_id = entity_id_new();

    transformInitIdentity(&player->transform);
    renderable_init(&player->renderable, &player->transform, "rom:/models/snake/snake.t3dm");

    assert(&player->renderable.skeleton != NULL && &player->renderable.model != NULL);
    player->transform.scale = (struct Vector3){0.125f, 0.125f, 0.125f};
    player->camera_transform = camera_transform;

    player->transform.position = definition->location;

    render_scene_add_renderable(&player->renderable, 2.0f);
    update_add(player, (update_callback)player_update, UPDATE_PRIORITY_PLAYER, UPDATE_LAYER_WORLD);

    player->look_direction = definition->rotation;

    vector2ComplexFromAngle(fixed_time_step * 7.0f, &player_max_rotation);

    // dynamic_object_init(
    //     entity_id,
    //     &player->collision,
    //     &player_collision,
    //     COLLISION_LAYER_TANGIBLE | COLLISION_LAYER_DAMAGE_PLAYER,
    //     &player->transform.position,
    //     &player->look_direction
    // );

    // player->collision.center.y = player_collision.data.capsule.inner_half_height + player_collision.data.capsule.radius;

    // collision_scene_add(&player->collision);

    player->animations.jump = t3d_anim_create(player->renderable.model, "Snake_Jump");
    t3d_anim_set_looping(&player->animations.jump, false); // don't loop this animation
    t3d_anim_set_playing(&player->animations.jump, false); // start in a paused state
    t3d_anim_attach(&player->animations.jump, &player->renderable.skeleton);

    player->animations.attack = t3d_anim_create(player->renderable.model, "Snake_Attack");
    t3d_anim_set_looping(&player->animations.attack, false); // don't loop this animation
    t3d_anim_set_playing(&player->animations.attack, false); // start in a paused state
    t3d_anim_attach(&player->animations.attack, &player->renderable.skeleton);

    player->animations.idle = t3d_anim_create(player->renderable.model, "Snake_Idle");
    t3d_anim_attach(&player->animations.idle, &player->renderable.skeleton);

    player->animations.walk = t3d_anim_create(player->renderable.model, "Snake_Walk");
    t3d_anim_attach(&player->animations.walk, &player->renderable.skeleton);

}

void player_destroy(struct player* player) {
    renderable_destroy(&player->renderable);

    render_scene_remove(&player->renderable);
    update_remove(player);
    // collision_scene_remove(&player->collision);
    t3d_anim_destroy(&player->animations.idle);
    t3d_anim_destroy(&player->animations.walk);
    t3d_anim_destroy(&player->animations.attack);
    t3d_anim_destroy(&player->animations.jump);
}