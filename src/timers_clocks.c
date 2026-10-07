#include "timers_clocks.h"

void timespec_add_microseconds(struct timespec *time, uint64_t microseconds)
{
    uint64_t nanoseconds = (uint64_t)time->tv_nsec
        + microseconds * (uint64_t)NANOSECONDS_PER_MICROSECOND;
    time->tv_sec += (time_t)(nanoseconds / (uint64_t)NANOSECONDS_PER_SECOND);
    time->tv_nsec = (long)(nanoseconds % (uint64_t)NANOSECONDS_PER_SECOND);
}

int64_t timespec_difference_microseconds(const struct timespec *start,
                                         const struct timespec *end)
{
    int64_t seconds = (int64_t)end->tv_sec - (int64_t)start->tv_sec;
    int64_t nanoseconds = (int64_t)end->tv_nsec - (int64_t)start->tv_nsec;
    return seconds * MICROSECONDS_PER_SECOND
        + nanoseconds / NANOSECONDS_PER_MICROSECOND;
}

int signal_timer_start(signal_timer_t *timer, const struct timespec *epoch,
                       uint64_t offset_us, uint64_t period_us, int signal_number)
{
    if (period_us == 0) {
        fprintf(stderr, "El periodo del temporizador debe ser mayor que cero.\n");
        return EINVAL;
    }

    memset(timer, 0, sizeof(*timer));
    sigemptyset(&timer->awaited_signal);
    sigaddset(&timer->awaited_signal, signal_number);

    struct sigevent notification;
    memset(&notification, 0, sizeof(notification));
    notification.sigev_notify = SIGEV_SIGNAL;
    notification.sigev_signo = signal_number;

    if (timer_create(TASK_CLOCK, &notification, &timer->id) != 0) {
        int error = errno;
        fprintf(stderr, "timer_create: %s\n", strerror(error));
        return error;
    }
    timer->active = true;

    struct itimerspec settings;
    memset(&settings, 0, sizeof(settings));
    settings.it_value = *epoch;
    timespec_add_microseconds(&settings.it_value, offset_us);
    settings.it_interval.tv_sec = (time_t)(period_us / MICROSECONDS_PER_SECOND);
    settings.it_interval.tv_nsec =
        (long)((period_us % MICROSECONDS_PER_SECOND)
               * NANOSECONDS_PER_MICROSECOND);

    if (timer_settime(timer->id, TIMER_ABSTIME, &settings, NULL) != 0) {
        int error = errno;
        fprintf(stderr, "timer_settime: %s\n", strerror(error));
        timer_delete(timer->id);
        timer->active = false;
        return error;
    }
    return 0;
}

int signal_timer_wait(signal_timer_t *timer)
{
    int received_signal = 0;
    int error = sigwait(&timer->awaited_signal, &received_signal);
    if (error != 0) {
        fprintf(stderr, "sigwait: %s\n", strerror(error));
    }
    return error;
}

int signal_timer_stop(signal_timer_t *timer)
{
    if (!timer->active) {
        return 0;
    }
    if (timer_delete(timer->id) != 0) {
        int error = errno;
        fprintf(stderr, "timer_delete: %s\n", strerror(error));
        timer->active = false;
        return error;
    }
    timer->active = false;
    return 0;
}

int clock_schedule_start(clock_schedule_t *schedule, const struct timespec *epoch,
                         uint64_t offset_us, uint64_t period_us)
{
    if (period_us == 0) {
        fprintf(stderr, "El periodo del reloj debe ser mayor que cero.\n");
        return EINVAL;
    }
    schedule->clock_id = TASK_CLOCK;
    schedule->deadline = *epoch;
    timespec_add_microseconds(&schedule->deadline, offset_us);
    schedule->period_us = period_us;
    return 0;
}

int clock_schedule_wait(clock_schedule_t *schedule)
{
    int error;
    do {
        error = clock_nanosleep(schedule->clock_id, TIMER_ABSTIME,
                                &schedule->deadline, NULL);
    } while (error == EINTR);

    if (error != 0) {
        fprintf(stderr, "clock_nanosleep: %s\n", strerror(error));
        return error;
    }
    timespec_add_microseconds(&schedule->deadline, schedule->period_us);
    return 0;
}
