#include "time.h"

#include <malloc.h>
#include <memory.h>
#include <libdragon.h>

#include "../util/flags.h"
#include "../util/blist.h"
#include "../util/callback_list.h"

struct update_element {
    void* data;
    short priority;
    short mask;
};

struct update_state {
    struct callback_list callbacks;
    struct callback_list fixed_callbacks;
    int enabled_layers;
};

#define MIN_UPDATE_CAPACITY    64

static struct update_state g_update_state;

uint64_t oldtime_ticks = 0;
uint32_t accumulator_ticks = 0;
uint64_t currtime_ticks = 0;
float currtime_sec = 0.0f;
uint32_t deltatime_ticks = 0;
float deltatime_sec = 0.0f;

void update_time() {

    currtime_ticks = get_ticks();
    currtime_sec = (float)TICKS_TO_MS(currtime_ticks) / 1000.0f;
    deltatime_ticks = currtime_ticks - oldtime_ticks;

    // Clamp deltatime to 0.25 seconds
    if (deltatime_ticks > TICKS_FROM_US(SEC_TO_USEC(0.25f)))
    {
        deltatime_ticks = TICKS_FROM_US(SEC_TO_USEC(0.25f));
    }
    oldtime_ticks = currtime_ticks;

    deltatime_sec = (float)TICKS_TO_MS((float)deltatime_ticks) / 1000.0f;
}

int update_compare_elements(void* a, void* b) {
    struct update_element* a_el = (struct update_element*)a;
    struct update_element* b_el = (struct update_element*)b;
    return a_el->priority - b_el->priority;
}

void update_reset() {
    callback_list_reset(&g_update_state.callbacks, sizeof(struct update_element), MIN_UPDATE_CAPACITY, update_compare_elements);
    callback_list_reset(&g_update_state.fixed_callbacks, sizeof(struct update_element), MIN_UPDATE_CAPACITY, update_compare_elements);
    g_update_state.enabled_layers = ~0;
}

void update_add(void* data, update_callback callback, int priority, int mask) {
    struct update_element element;

    element.data = data;
    element.priority = priority;
    element.mask = mask;

    callback_list_insert_with_id(&g_update_state.callbacks, callback, &element, (callback_id)data);
}

void fixed_update_add(void* data, update_callback callback, int priority, int mask) {
    struct update_element element;

    element.data = data;
    element.priority = priority;
    element.mask = mask;

    callback_list_insert_with_id(&g_update_state.fixed_callbacks, callback, &element, (callback_id)data);
}

void update_remove(void* data) {
    callback_list_remove(&g_update_state.callbacks, (callback_id)data);
}

void fixed_update_remove(void* data) {
    callback_list_remove(&g_update_state.fixed_callbacks, (callback_id)data);
}

void update_pause_layers(int mask) {
    CLEAR_FLAG(g_update_state.enabled_layers, mask);
}

void update_unpause_layers(int mask) {
    SET_FLAG(g_update_state.enabled_layers, mask);
}

bool update_has_layer(int mask) {
    return mask & g_update_state.enabled_layers;
}

void update_dispatch() {

    callback_list_begin(&g_update_state.callbacks);

    struct callback_element* current = callback_list_get(&g_update_state.callbacks, 0);

    for (int i = 0; i < g_update_state.callbacks.count; ++i) {
        struct update_element* element = callback_element_get_data(current);

        if (element->mask & g_update_state.enabled_layers) {
            ((update_callback)current->callback)(element->data);
        }
        
        current = callback_list_next(&g_update_state.callbacks, current);
    }

    callback_list_end(&g_update_state.callbacks);
}

void fixed_update_dispatch() {
    callback_list_begin(&g_update_state.fixed_callbacks);

    struct callback_element* current = callback_list_get(&g_update_state.fixed_callbacks, 0);

    for (int i = 0; i < g_update_state.fixed_callbacks.count; ++i) {
        struct update_element* element = callback_element_get_data(current);

        if (element->mask & g_update_state.enabled_layers) {
            ((update_callback)current->callback)(element->data);
        }
        
        current = callback_list_next(&g_update_state.fixed_callbacks, current);
    }

    callback_list_end(&g_update_state.fixed_callbacks);
}