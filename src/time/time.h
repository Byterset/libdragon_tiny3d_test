#ifndef __TIME_TIME_H__
#define __TIME_TIME_H__

#include <stdint.h>
#include <stdbool.h>

typedef void (*update_callback)(void* data);

typedef int update_id;

#define PHYSICS_TICKRATE 60.0f
#define FIXED_DELTATIME (1.0f/PHYSICS_TICKRATE)
#define FIXED_DELTATIME_SQUARED (FIXED_DELTATIME * FIXED_DELTATIME)

#define SEC_TO_USEC(a) (((double)a) * 1000000.0f)


#define UPDATE_LAYER_WORLD          (1 << 0)
#define UPDATE_LAYER_PLAYER         (1 << 1)
#define UPDATE_LAYER_DIALOG         (1 << 2)
#define UPDATE_LAYER_PAUSE_MENU     (1 << 3)

#define UPDATE_PRIORITY_PLAYER  0
#define UPDATE_PRIORITY_WORLD   1
#define UPDATE_PRIORITY_EFFECTS  1
#define UPDATE_PRIORITY_CAMERA  2

extern uint64_t oldtime_ticks;
extern uint32_t accumulator_ticks;
extern uint64_t currtime_ticks;
extern uint32_t frametime_ticks;
extern float currtime_sec;
extern float frametime_sec;

void update_reset();
void update_time();
void update_add(void* data, update_callback callback, int priority, int mask);
void fixed_update_add(void* data, update_callback callback, int priority, int mask);
void update_remove(void* data);
void fixed_update_remove(void* data);
void update_remove_with_data(void* data, update_callback callback);
void fixed_update_remove_with_data(void* data, update_callback callback);

void update_pause_layers(int mask);
void update_unpause_layers(int mask);
bool update_has_layer(int mask);

void update_dispatch();


void fixed_update_dispatch();




#endif