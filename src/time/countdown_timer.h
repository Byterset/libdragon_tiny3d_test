#ifndef __TIME_COUNTDOWN_TIMER_H__
#define __TIME_COUNTDOWN_TIMER_H__

#include <stdint.h>
#include <stdbool.h>
#include "time.h"

typedef void (*countdown_timer_callback)(void* data);

struct CountdownTimer {
    float current_time;
    float duration_sec;
    bool is_running;
    countdown_timer_callback on_timer_stop;
    countdown_timer_callback on_timer_start;
    void* data;
};


void countdown_timer_init(struct CountdownTimer* timer, float duration_sec, countdown_timer_callback on_timer_stop, countdown_timer_callback on_timer_start, void* data);
void countdown_timer_start(struct CountdownTimer* timer);
void countdown_timer_stop(struct CountdownTimer* timer);
void countdown_timer_reset(struct CountdownTimer* timer, bool is_running);
void countdown_timer_update(struct CountdownTimer* timer);
void countdown_timer_pause(struct CountdownTimer* timer);
void countdown_timer_resume(struct CountdownTimer* timer);
float countdown_timer_get_progress(struct CountdownTimer* timer);

#endif