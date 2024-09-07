#ifndef __SPELL_FIRE_H__
#define __SPELL_FIRE_H__

#include <t3d/t3dmodel.h>
#include "../math/vector3.h"
#include "../math/vector2.h"

#define MAX_FIRE_PARTICLE_COUNT     8

struct fire {
    struct Vector3 particle_offset[MAX_FIRE_PARTICLE_COUNT];
    struct Vector3 position;
    struct Vector2 rotation;
    float cycle_time;
    float total_time;
    float end_time;
    uint16_t index_offset;
    T3DMaterial* material;
    rspq_block_t* mat_block;
};

void fire_init(struct fire* fire);
void fire_destroy(struct fire* fire);

void fire_update(struct fire* fire);

#endif