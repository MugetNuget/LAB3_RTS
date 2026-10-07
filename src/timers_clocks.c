#include "timers_clocks.h"

static struct timespec g_sim_start_time;
static bool g_sim_start_recorded = false;

void timespec_add_us(struct timespec *t, uint64_t delta_us)
{
    uint64_t delta_ns = delta_us * NSEC_PER_USEC;
    delta_ns += t->tv_nsec;

    t->tv_sec += delta_ns / NSEC_PER_SEC;
    t->tv_nsec = delta_ns % NSEC_PER_SEC;
}

int64_t timespec_diff_us(const struct timespec *start, const struct timespec *end)
{
    int64_t sec_diff  = (int64_t)(end->tv_sec - start->tv_sec);
    int64_t nsec_diff = (int64_t)(end->tv_nsec - start->tv_nsec);
    return (sec_diff * (int64_t)USEC_PER_SEC) + (nsec_diff / (int64_t)NSEC_PER_USEC);
}

double get_elapsed_time_sec(void)
{
    struct timespec now;
    clock_gettime(RT_CLOCK_SOURCE, &now);

    if (!g_sim_start_recorded) {
        g_sim_start_time = now;
        g_sim_start_recorded = true;
        return 0.0;
    }

    return (double)(now.tv_sec - g_sim_start_time.tv_sec) +
           (double)(now.tv_nsec - g_sim_start_time.tv_nsec) / 1e9;
}

/* Inicializa y arma un temporizador POSIX */
posix_timer_task_t* posix_timer_create_and_arm(uint64_t offset_us, uint64_t period_us, int signo)
{
    posix_timer_task_t *task = malloc(sizeof(posix_timer_task_t));
    if (!task) {
        perror("malloc");
        return NULL;
    }

    task->signo     = signo;
    task->period_us = period_us;
    task->offset_us = offset_us;

    sigemptyset(&task->sigset);
    sigaddset(&task->sigset, signo);

    memset(&task->sigev, 0, sizeof(struct sigevent));
    task->sigev.sigev_notify = SIGEV_SIGNAL;
    task->sigev.sigev_signo  = signo;

    int ret = timer_create(RT_CLOCK_SOURCE, &task->sigev, &task->timer_id);
    if (ret != 0) {
        perror("timer_create");
        free(task);
        return NULL;
    }

    memset(&task->its, 0, sizeof(struct itimerspec));
    
    if (offset_us == 0) {
        task->its.it_value.tv_sec  = 0;
        task->its.it_value.tv_nsec = 1000;
    } else {
        task->its.it_value.tv_sec  = offset_us / USEC_PER_SEC;
        task->its.it_value.tv_nsec = (offset_us % USEC_PER_SEC) * NSEC_PER_USEC;
    }

    task->its.it_interval.tv_sec  = period_us / USEC_PER_SEC;
    task->its.it_interval.tv_nsec = (period_us % USEC_PER_SEC) * NSEC_PER_USEC;

    ret = timer_settime(task->timer_id, 0, &task->its, NULL);
    if (ret != 0) {
        perror("timer_settime");
        timer_delete(task->timer_id);
        free(task);
        return NULL;
    }

    return task;
}

/* Espera la expiracion del temporizador con sigwait */
void posix_timer_wait_activation(posix_timer_task_t *task)
{
    int sig_received = 0;
    int ret = sigwait(&task->sigset, &sig_received);
    if (ret != 0) {
        perror("sigwait");
    }
}

void posix_timer_destroy(posix_timer_task_t *task)
{
    if (task) {
        timer_delete(task->timer_id);
        free(task);
    }
}

/* Inicializa tarea periodica con reloj POSIX */
posix_clock_task_t* posix_clock_init_task(uint64_t offset_us, uint64_t period_us, clockid_t clock_id)
{
    posix_clock_task_t *task = malloc(sizeof(posix_clock_task_t));
    if (!task) {
        perror("malloc");
        return NULL;
    }

    task->clock_id  = clock_id;
    task->period_us = period_us;
    task->offset_us = offset_us;

    int ret = clock_gettime(clock_id, &task->next_activation);
    if (ret != 0) {
        perror("clock_gettime");
        free(task);
        return NULL;
    }

    timespec_add_us(&task->next_activation, offset_us);
    return task;
}

/* Espera la siguiente activacion con clock_nanosleep absoluto */
void posix_clock_wait_activation(posix_clock_task_t *task)
{
    int ret;
    do {
        ret = clock_nanosleep(task->clock_id, TIMER_ABSTIME, &task->next_activation, NULL);
    } while (ret == EINTR);

    if (ret != 0 && ret != EINTR) {
        perror("clock_nanosleep");
    }

    timespec_add_us(&task->next_activation, task->period_us);
}

void posix_clock_destroy(posix_clock_task_t *task)
{
    if (task) {
        free(task);
    }
}
