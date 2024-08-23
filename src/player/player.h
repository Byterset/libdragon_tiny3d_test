#ifndef __SCENE_PLAYER_H__
#define __SCENE_PLAYER_H__

#include "../math/transform.h"
#include "../math/vector2.h"
#include "../render/render_batch.h"
#include "../render/renderable.h"
#include "../collision/dynamic_object.h"
#include <t3d/t3danim.h>

#define PLAYER_MOVE_SPEED 50.0f
#define PLAYER_TURN_SPEED 20.0f


struct player_animations {
    T3DAnim idle;
    T3DAnim walk;
    T3DAnim attack;
    T3DAnim jump;
};

struct player_definition {
    struct Vector3 location;
    struct Vector2 rotation;
};

struct player {
    struct Transform transform;
    struct renderable renderable;
    struct Transform* camera_transform;
    struct Vector2 look_direction;
    bool is_attacking;
    bool is_jumping;
    struct dynamic_object collision;
    T3DSkeleton skelBlend;
    struct player_animations animations;
};

void player_init(struct player* player, struct player_definition* definition, struct Transform* camera_transform);

void player_render(struct player* player, struct render_batch* batch);

void player_destroy(struct player* player);

#endif