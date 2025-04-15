#include "fire.h"

#include "../render/render_scene.h"
#include "../time/time.h"
#include "../math/mathf.h"


#define CYCLE_TIME  0.32f

#define FIRE_LENGTH         2.5f

#define MAX_RADIUS          1.3f
#define MAX_RANDOM_OFFSET   0.3f

#define START_FADE          0.7f

#define TIP_RISE            0.6f

#define INITIAL_ALPHA       200


void fire_apply_transform(struct fire* fire) {
    // fire->position = (Vector3){{-2.0f, 1.0f, 0.0f}};

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

    struct material* material = material_cache_load("rom:/materials/spell/fire_particle.mat");

    struct render_batch_billboard_element* element = render_batch_add_particles(batch, material, particle_count);

    float time_lerp = fire->cycle_time * (1.0f / CYCLE_TIME);

    for (int i = 0; i < element->sprite_count; i += 1) {
        struct render_billboard_sprite* sprite = &element->sprites[i];

        float particle_time = (i + particle_offset + time_lerp) * (1.0f / MAX_FIRE_PARTICLE_COUNT);

        sprite->color.r = 255;
        sprite->color.g = 255;
        sprite->color.b = 255;
        sprite->color.a = INITIAL_ALPHA;

        sprite->radius = particle_time * MAX_RADIUS;

        int final_index = i + fire->index_offset;

        if (final_index >= MAX_FIRE_PARTICLE_COUNT) {
            final_index -= MAX_FIRE_PARTICLE_COUNT;
        }
        vector3AddScaled(&fire->position, &(Vector3){{0, 1, 0}}, particle_time * FIRE_LENGTH, &sprite->position);
        vector3AddScaled(&sprite->position, &fire->particle_offset[final_index], particle_time, &sprite->position);

        if (particle_time > START_FADE) {
            float alpha =  1- (particle_time - START_FADE) * (1.0f / (1.0f - START_FADE));
            // sprite->color.a = (uint8_t)(randomInRange(0, 256));
            sprite->color.a = (uint8_t)(alpha * INITIAL_ALPHA);
            sprite->position.y += TIP_RISE * (1.0f - alpha);
        }
    }
}

void fire_init(struct fire* fire) {
    render_scene_add_callback(NULL, 3.0f, (render_scene_callback)fire_render, fire);

    fire->cycle_time = 0.0f;
    fire->total_time = 0.0f;
    fire->end_time = -1.0f;

    fire->index_offset = 0;

    fire_apply_transform(fire);

    for (int i = 0; i < MAX_FIRE_PARTICLE_COUNT; i += 1) {
        fire->particle_offset[i] = gZeroVec;

        Vector3* offset = &fire->particle_offset[fire->index_offset];
        offset->x = randomInRangef(-MAX_RANDOM_OFFSET, MAX_RANDOM_OFFSET);
        offset->y = randomInRangef(-MAX_RANDOM_OFFSET, MAX_RANDOM_OFFSET);
        offset->z = randomInRangef(-MAX_RANDOM_OFFSET, MAX_RANDOM_OFFSET);
    }
    update_add(fire, (update_callback)fire_update, UPDATE_PRIORITY_EFFECTS, UPDATE_LAYER_WORLD);

}

void fire_destroy(struct fire* fire) {
    render_scene_remove(fire);
}

void fire_update(struct fire* fire) {
    fire->cycle_time += deltatime_sec;
    fire->total_time += deltatime_sec;

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