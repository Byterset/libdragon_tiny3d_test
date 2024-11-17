#include "countdown_timer.h"

void countdown_timer_init(struct CountdownTimer* timer, float duration_sec, countdown_timer_callback on_timer_stop, countdown_timer_callback on_timer_start, void* data){
    timer->duration_sec = duration_sec;
    timer->current_time = 0.0f;
    timer->on_timer_stop = on_timer_stop;
    timer->on_timer_start = on_timer_start;
    timer->data = data;
    timer->is_running = false;
}

void countdown_timer_start(struct CountdownTimer* timer) {
    timer->is_running = true;
    if (timer->on_timer_start) {
        timer->on_timer_start(timer->data);
    }
    update_add(timer, (update_callback)countdown_timer_update, UPDATE_PRIORITY_WORLD, UPDATE_LAYER_WORLD);
}

void countdown_timer_pause(struct CountdownTimer* timer) {
    timer->is_running = false;
}

void countdown_timer_resume(struct CountdownTimer* timer) {
    timer->is_running = true;
}

void countdown_timer_stop(struct CountdownTimer* timer) {
    timer->is_running = false;
    if (timer->on_timer_stop) {
        timer->on_timer_stop(timer->data);
    }
    update_remove(timer);
}

void countdown_timer_update(struct CountdownTimer* timer) {
    if (timer->is_running) {
        timer->current_time += deltatime_sec;
        if (timer->current_time >= timer->duration_sec) {
            countdown_timer_stop(timer);
        }
    }
}

void countdown_timer_reset(struct CountdownTimer* timer, bool is_running) {
    timer->current_time = 0.0f;
    timer->is_running = is_running;
}

float countdown_timer_get_progress(struct CountdownTimer* timer) {
    return timer->current_time / timer->duration_sec;
}