#ifndef __SCENE_PLATFORM_H__
#define __SCENE_PLATFORM_H__

#include "../../math/transform.h"
#include "../../math/vector2.h"
#include "../../render/render_batch.h"
#include "../../render/renderable.h"
#include "../../render/model.h"
#include "../../collision/dynamic_object.h"
#include "../../scene/scene_definition.h"

struct platform {
    struct Transform transform;
    struct renderable renderable;
    struct dynamic_object collision;
    struct Vector2 look_direction;
};

void platform_init(struct platform* platform, struct platform_definition* def);

void platform_destroy(struct platform* platform);

#endif