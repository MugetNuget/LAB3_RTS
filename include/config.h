#ifndef CONFIG_H
#define CONFIG_H

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <sched.h>

#define NSEC_PER_SEC            1000000000ULL
#define USEC_PER_SEC            1000000ULL
#define NSEC_PER_USEC           1000ULL

/* Parametros temporales de las tareas */
#define PERIOD_TAU1_US          20000
#define OFFSET_TAU1_US          10000

#define PERIOD_TAU2_US          40000
#define OFFSET_TAU2_US          20000

#define PERIOD_TAU3_US          80000
#define OFFSET_TAU3_US          30000

#define PERIOD_TAU4_US          160000
#define OFFSET_TAU4_US          40000

/* Senales de tiempo real para temporizadores POSIX */
#define SIG_TAU1_ESC            (SIGRTMIN + 1)
#define SIG_TAU2_TCS            (SIGRTMIN + 2)

#define RT_CLOCK_SOURCE         CLOCK_MONOTONIC
#define DEFAULT_SIM_TIME_SEC    10
#define TELEMETRY_BUFFER_SIZE   32

#endif /* CONFIG_H */
