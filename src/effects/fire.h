#ifndef __EFFECT_FIRE_H__
#define __EFFECT_FIRE_H__

#include <t3d/t3dmodel.h>
#include "../math/vector3.h"
#include "../math/vector2.h"
#include "../render/material.h"
#include "../resource/material_cache.h"

#define MAX_FIRE_PARTICLE_COUNT     7

struct fire {
    Vector3 particle_offset[MAX_FIRE_PARTICLE_COUNT];
    Vector3 position;
    Vector2 rotation;
    float cycle_time;
    float total_time;
    float end_time;
    uint16_t index_offset;
};

void fire_init(struct fire* fire);
void fire_destroy(struct fire* fire);

void fire_update(struct fire* fire);

#endif