#ifndef TIMERS_CLOCKS_H
#define TIMERS_CLOCKS_H

#include "config.h"

typedef struct {
    timer_t id;
    sigset_t awaited_signal;
    bool active;
} signal_timer_t;

typedef struct {
    clockid_t clock_id;
    struct timespec deadline;
    uint64_t period_us;
} clock_schedule_t;

int signal_timer_start(signal_timer_t *timer, const struct timespec *epoch,
                       uint64_t offset_us, uint64_t period_us, int signal_number);
int signal_timer_wait(signal_timer_t *timer);
int signal_timer_stop(signal_timer_t *timer);

int clock_schedule_start(clock_schedule_t *schedule, const struct timespec *epoch,
                         uint64_t offset_us, uint64_t period_us);
int clock_schedule_wait(clock_schedule_t *schedule);

void timespec_add_microseconds(struct timespec *time, uint64_t microseconds);
int64_t timespec_difference_microseconds(const struct timespec *start,
                                         const struct timespec *end);

#endif
