#include "fire.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../math/mathf.h"


#define CYCLE_TIME  0.08f

#define FIRE_LENGTH         4.0f

#define MAX_RADIUS          0.5f
#define MAX_RANDOM_OFFSET   0.3f

#define START_FADE          0.75f

#define TIP_RISE            0.5f


void fire_apply_transform(struct fire* fire) {
    fire->position = (struct Vector3){0.0f, 2.0f, 0.0f};

    fire->rotation = gZeroVec2;
}

void fire_render(struct fire* fire, struct render_batch* batch) {
    int particle_count = (int)(fire->total_time * (1.0f / CYCLE_TIME));
    int particle_offset = 0;

    if (particle_count > MAX_FIRE_PARTICLE_COUNT) {
        particle_count = MAX_FIRE_PARTICLE_COUNT;
    }

    if (fire->end_time != -1.0f) {
        particle_offset = (fire->total_time - fire->end_time) * (1.0f / CYCLE_TIME);

        if (particle_offset > particle_count) {
            return;
        }

        particle_count -= particle_offset;
    }

    // struct render_batch_billboard_element* element = render_batch_add_particles(batch, fire->material->textureA.texture, fire->mat_block, particle_count);

    // float time_lerp = fire->cycle_time * (1.0f / CYCLE_TIME);

    // for (int i = 0; i < element->sprite_count; i += 1) {
    //     struct render_billboard_sprite* sprite = &element->sprites[i];

    //     float particle_time = (i + particle_offset + time_lerp) * (1.0f / MAX_FIRE_PARTICLE_COUNT);

    //     sprite->color.r = 255;
    //     sprite->color.g = 255;
    //     sprite->color.b = 255;
    //     sprite->color.a = 255;

    //     sprite->radius = particle_time * MAX_RADIUS;

    //     int final_index = i + fire->index_offset;

    //     if (final_index >= MAX_FIRE_PARTICLE_COUNT) {
    //         final_index -= MAX_FIRE_PARTICLE_COUNT;
    //     }

    //     vector3AddScaled(&sprite->position, &fire->particle_offset[final_index], particle_time, &sprite->position);

    //     if (particle_time > START_FADE) {
    //         float alpha = 1.0f - (particle_time - START_FADE) * (1.0f / (1.0f - START_FADE));

    //         sprite->color.a = (uint8_t)(alpha * 255);
    //         sprite->position.y += TIP_RISE * (1.0f - alpha);
    //     }
    // }
}

void fire_init(struct fire* fire) {
    render_scene_add(&gZeroVec, 4.0f, (render_scene_callback)fire_render, fire);

    // fire->material = t3d_material_create()
    fire->cycle_time = 0.0f;
    fire->total_time = 0.0f;
    fire->end_time = -1.0f;

    fire->index_offset = 0;

    fire_apply_transform(fire);

    for (int i = 0; i < MAX_FIRE_PARTICLE_COUNT; i += 1) {
        fire->particle_offset[i] = gZeroVec;

        struct Vector3* offset = &fire->particle_offset[fire->index_offset];
        offset->x = randomInRangef(-MAX_RANDOM_OFFSET, MAX_RANDOM_OFFSET);
        offset->y = randomInRangef(-MAX_RANDOM_OFFSET, MAX_RANDOM_OFFSET);
        offset->z = randomInRangef(-MAX_RANDOM_OFFSET, MAX_RANDOM_OFFSET);
    }

}

void fire_destroy(struct fire* fire) {
    render_scene_remove(fire);

}

void fire_update(struct fire* fire) {
    fire->cycle_time += frametime_sec;
    fire->total_time += frametime_sec;

    if (fire->cycle_time > CYCLE_TIME) {
        fire->cycle_time -= CYCLE_TIME;

        if (fire->index_offset == 0) {
            fire->index_offset = MAX_FIRE_PARTICLE_COUNT - 1;
        } else {
            fire->index_offset -= 1;
        }
    }

    fire_apply_transform(fire);
}