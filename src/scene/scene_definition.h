#ifndef __SCENE_SCENE_DEFINITION_H__
#define __SCENE_SCENE_DEFINITION_H__

#include "../math/vector3.h"
#include "../math/vector2.h"
// #include "../cutscene/expression.h"
#include <stdint.h>

struct crate_definition {
    Vector3 position;
    Vector2 rotation;
};

enum collectable_type {
    COLLECTABLE_TYPE_COIN,
};


struct collectable_definition {
    Vector3 position;
    Vector2 rotation;
    enum collectable_type collectable_type;
    uint32_t collectable_sub_type;
};

struct coin_definition {
    Vector3 position;
};

struct training_dummy_definition {
    Vector3 position;
    Vector2 rotation;
};

struct box_definition {
    Vector3 position;
};

struct cone_definition {
    Vector3 position;
};

struct cylinder_definition {
    Vector3 position;
};

struct platform_definition {
    Vector3 position;
};


enum npc_type {
    NPC_TYPE_NONE,
    NPC_TYPE_PLAYER,
    NPC_TYPE_SUBJECT,
    NPC_TYPE_MENTOR,
};

enum interaction_type {
    INTERACTION_NONE,

    INTERACTION_LOOK,
    INTERACTION_MOVE,
    INTERACTION_LOOK_MOVE,
    INTERACTION_SPACE,
    INTERACTION_LOOK_SPACE,
    INTERACTION_MOVE_SPACE,
    INTERACTION_LOOK_MOVE_SPACE,

    INTERACTION_WAIT,
    INTERACTION_LOOK_WAIT,
    INTERACTION_MOVE_WAIT,
    INTERACTION_LOOK_MOVE_WAIT,
    INTERACTION_SPACE_WAIT,
    INTERACTION_LOOK_SPACE_WAIT,
    INTERACTION_MOVE_SPACE_WAIT,
    INTERACTION_LOOK_MOVE_SPACE_WAIT,
};

struct npc_definition {
    Vector3 position;
    Vector2 rotation;
    enum npc_type npc_type;
    char* dialog;
};

#endif