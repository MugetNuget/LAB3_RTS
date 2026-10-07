#ifndef TIMERS_CLOCKS_H
#define TIMERS_CLOCKS_H

#include "config.h"

/* Estructura para tareas periodicas con temporizadores POSIX */
typedef struct posix_timer_task {
    timer_t             timer_id;
    struct sigevent     sigev;
    struct itimerspec   its;
    sigset_t            sigset;
    int                 signo;
    uint64_t            period_us;
    uint64_t            offset_us;
} posix_timer_task_t;

/* Estructura para tareas periodicas con relojes POSIX */
typedef struct posix_clock_task {
    clockid_t           clock_id;
    struct timespec     next_activation;
    uint64_t            period_us;
    uint64_t            offset_us;
} posix_clock_task_t;

/* Prototipos de funciones */
posix_timer_task_t* posix_timer_create_and_arm(uint64_t offset_us, uint64_t period_us, int signo);
void posix_timer_wait_activation(posix_timer_task_t *task);
void posix_timer_destroy(posix_timer_task_t *task);

posix_clock_task_t* posix_clock_init_task(uint64_t offset_us, uint64_t period_us, clockid_t clock_id);
void posix_clock_wait_activation(posix_clock_task_t *task);
void posix_clock_destroy(posix_clock_task_t *task);

void timespec_add_us(struct timespec *t, uint64_t delta_us);
int64_t timespec_diff_us(const struct timespec *start, const struct timespec *end);
double get_elapsed_time_sec(void);

#endif /* TIMERS_CLOCKS_H */
