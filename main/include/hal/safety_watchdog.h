#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int diag_left_io;
    int diag_right_io;
    volatile bool tripped;
    volatile uint32_t trip_count;
} safety_watchdog_t;

void safety_watchdog_init(safety_watchdog_t *watchdog);
bool safety_watchdog_is_tripped(const safety_watchdog_t *watchdog);
void safety_watchdog_clear(safety_watchdog_t *watchdog);

#endif