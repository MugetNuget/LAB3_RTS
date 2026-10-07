#include "tasks.h"
#include "timers_clocks.h"

typedef struct {
    const char *label;
    const char *mechanism;
    void *(*entry)(void *);
    uint64_t period_us;
    uint64_t offset_us;
} task_definition_t;

static const task_definition_t task_definitions[TASK_COUNT] = {
    {"ESC / estabilidad", "timer + SIGRTMIN+1", task_stability,
     ESC_PERIOD_US, ESC_OFFSET_US},
    {"TCS / traccion", "timer + SIGRTMIN+2", task_traction,
     TCS_PERIOD_US, TCS_OFFSET_US},
    {"Inyeccion y velocidad", "CLOCK_MONOTONIC", task_fuel,
     FUEL_PERIOD_US, FUEL_OFFSET_US},
    {"Diagnostico", "CLOCK_MONOTONIC", task_diagnostics,
     DIAGNOSTICS_PERIOD_US, DIAGNOSTICS_OFFSET_US}
};

static int task_stop_signal(unsigned task_index)
{
    if (task_index == 0) {
        return ESC_TIMER_SIGNAL;
    }
    if (task_index == 1) {
        return TCS_TIMER_SIGNAL;
    }
    return 0;
}

static int parse_duration(int argc, char **argv, unsigned *duration)
{
    *duration = DEFAULT_RUN_SECONDS;
    if (argc < 2) {
        return 0;
    }

    char *end = NULL;
    errno = 0;
    long parsed = strtol(argv[1], &end, 10);
    if (errno != 0 || end == argv[1] || *end != '\0'
        || parsed < 1 || parsed > MAX_RUN_SECONDS) {
        fprintf(stderr, "Duracion invalida. Use un entero entre 1 y %d segundos.\n",
                MAX_RUN_SECONDS);
        return -1;
    }
    *duration = (unsigned)parsed;
    return 0;
}

static void print_configuration(unsigned duration)
{
    printf("Control automotriz simulado con tareas POSIX periodicas\n");
    printf("Duracion: %u s | reloj base: CLOCK_MONOTONIC\n", duration);
    printf("%-4s %-27s %-24s %10s %10s\n",
           "ID", "Funcion", "Mecanismo", "Periodo", "Offset");
    for (unsigned i = 0; i < TASK_COUNT; ++i) {
        printf("Tau%u %-27s %-24s %8llu ms %8llu ms\n",
               i + 1, task_definitions[i].label, task_definitions[i].mechanism,
               (unsigned long long)(task_definitions[i].period_us / 1000U),
               (unsigned long long)(task_definitions[i].offset_us / 1000U));
    }
    printf("Estado inicial: velocidad=54 km/h, par=72%%, combustible=58%%\n\n");
}

static int sleep_until(const struct timespec *deadline)
{
    int error;
    do {
        error = clock_nanosleep(TASK_CLOCK, TIMER_ABSTIME, deadline, NULL);
    } while (error == EINTR);
    if (error != 0) {
        fprintf(stderr, "clock_nanosleep(fin de simulacion): %s\n",
                strerror(error));
    }
    return error;
}

static void print_results(vehicle_t *vehicle)
{
    printf("\nResumen de activaciones y variacion temporal\n");
    printf("%-4s %-27s %10s %14s %14s\n",
           "ID", "Funcion", "Activaciones", "Periodo medio", "Jitter max.");
    for (unsigned i = 0; i < TASK_COUNT; ++i) {
        const release_stats_t *stats = &vehicle->stats[i];
        double mean_ms = stats->releases > 1
            ? (double)(stats->accumulated_period_us
                       / (long double)(stats->releases - 1U)) / 1000.0
            : 0.0;
        printf("Tau%u %-27s %12llu %11.3f ms %10lld us\n",
               i + 1, task_definitions[i].label,
               (unsigned long long)stats->releases,
               mean_ms,
               (long long)stats->greatest_jitter_us);
    }

    int error = pthread_mutex_lock(&vehicle->mutex);
    if (error != 0) {
        fprintf(stderr, "pthread_mutex_lock(resumen): %s\n", strerror(error));
        return;
    }
    printf("\nEstado final: freno=%.1f%%, par=%.1f%%, combustible=%.1f%%, "
           "velocidad=%.1f km/h; aceleracion lateral=%.2fg, patinamiento=%.1f%%\n",
           vehicle->signals[SIGNAL_BRAKE],
           vehicle->signals[SIGNAL_DRIVE_TORQUE],
           vehicle->signals[SIGNAL_FUEL_RATE],
           vehicle->signals[SIGNAL_SPEED],
           vehicle->lateral_acceleration_g,
           vehicle->wheel_slip_percent);
    pthread_mutex_unlock(&vehicle->mutex);
}

int main(int argc, char **argv)
{
    unsigned duration;
    if (parse_duration(argc, argv, &duration) != 0) {
        return EXIT_FAILURE;
    }

    vehicle_t vehicle;
    if (vehicle_init(&vehicle) != 0) {
        return EXIT_FAILURE;
    }

    sigset_t timer_signals;
    sigemptyset(&timer_signals);
    sigaddset(&timer_signals, ESC_TIMER_SIGNAL);
    sigaddset(&timer_signals, TCS_TIMER_SIGNAL);
    int error = pthread_sigmask(SIG_BLOCK, &timer_signals, NULL);
    if (error != 0) {
        fprintf(stderr, "pthread_sigmask: %s\n", strerror(error));
        vehicle_destroy(&vehicle);
        return EXIT_FAILURE;
    }

    if (clock_gettime(TASK_CLOCK, &vehicle.epoch) != 0) {
        perror("clock_gettime(epoch)");
        vehicle_destroy(&vehicle);
        return EXIT_FAILURE;
    }
    timespec_add_microseconds(&vehicle.epoch, 100000U);
    print_configuration(duration);

    pthread_t threads[TASK_COUNT];
    task_context_t contexts[TASK_COUNT];
    unsigned started = 0;
    for (; started < TASK_COUNT; ++started) {
        contexts[started].vehicle = &vehicle;
        contexts[started].task_index = started;
        contexts[started].failed = false;
        error = pthread_create(&threads[started], NULL,
                               task_definitions[started].entry,
                               &contexts[started]);
        if (error != 0) {
            fprintf(stderr, "pthread_create(Tau%u): %s\n",
                    started + 1, strerror(error));
            break;
        }
    }

    if (started != TASK_COUNT) {
        vehicle_stop(&vehicle);
        for (unsigned i = 0; i < started; ++i) {
            int stop_signal = task_stop_signal(i);
            if (stop_signal != 0) {
                int signal_error = pthread_kill(threads[i], stop_signal);
                if (signal_error != 0) {
                    fprintf(stderr, "pthread_kill(Tau%u): %s\n",
                            i + 1, strerror(signal_error));
                }
            }
        }
        for (unsigned i = 0; i < started; ++i) {
            pthread_join(threads[i], NULL);
        }
        vehicle_destroy(&vehicle);
        return EXIT_FAILURE;
    }

    struct timespec finish = vehicle.epoch;
    finish.tv_sec += (time_t)duration;
    if (sleep_until(&finish) != 0) {
        vehicle_stop(&vehicle);
        for (unsigned i = 0; i < TASK_COUNT; ++i) {
            int stop_signal = task_stop_signal(i);
            if (stop_signal != 0) {
                pthread_kill(threads[i], stop_signal);
            }
        }
        for (unsigned i = 0; i < TASK_COUNT; ++i) {
            pthread_join(threads[i], NULL);
        }
        vehicle_destroy(&vehicle);
        return EXIT_FAILURE;
    }

    vehicle_stop(&vehicle);
    for (unsigned i = 0; i < TASK_COUNT; ++i) {
        int stop_signal = task_stop_signal(i);
        if (stop_signal != 0) {
            error = pthread_kill(threads[i], stop_signal);
            if (error != 0) {
                fprintf(stderr, "pthread_kill(Tau%u): %s\n",
                        i + 1, strerror(error));
            }
        }
    }
    bool tasks_failed = false;
    for (unsigned i = 0; i < TASK_COUNT; ++i) {
        error = pthread_join(threads[i], NULL);
        if (error != 0) {
            fprintf(stderr, "pthread_join(Tau%u): %s\n",
                    i + 1, strerror(error));
            tasks_failed = true;
        }
        tasks_failed = tasks_failed || contexts[i].failed;
    }

    print_results(&vehicle);
    vehicle_destroy(&vehicle);
    return tasks_failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
