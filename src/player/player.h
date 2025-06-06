#ifndef __SCENE_PLAYER_H__
#define __SCENE_PLAYER_H__

#include "../math/transform.h"
#include "../math/vector2.h"
#include "../render/render_batch.h"
#include "../render/renderable.h"
#include "../render/model.h"
#include "../collision/physics_object.h"
#include <t3d/t3danim.h>
#include <stdbool.h>

struct player_animations {
    T3DAnim idle;
    T3DAnim walk;
    T3DAnim attack;
    T3DAnim jump;
};

struct player_definition {
    Vector3 location;
    Vector2 rotation;
};

struct player {
    Transform transform;
    struct renderable renderable;
    Transform* camera_transform;
    Vector2 look_direction;
    bool is_attacking;
    bool is_jumping;
    struct physics_object physics;
    T3DSkeleton skelBlend;
    struct player_animations animations;
    Vector3 desired_velocity;
    bool is_on_ground;
    raycast_hit ray_down_hit;
    raycast_hit ray_fwd_hit;
    Vector3 ground_normal;
};

void player_init(struct player* player, struct player_definition* definition, Transform* camera_transform);

void player_destroy(struct player* player);

#endif