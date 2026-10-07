#ifndef CONFIG_H
#define CONFIG_H

#define _GNU_SOURCE

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NANOSECONDS_PER_SECOND 1000000000L
#define MICROSECONDS_PER_SECOND 1000000L
#define NANOSECONDS_PER_MICROSECOND 1000L

#define TASK_COUNT 4
#define LOG_CAPACITY 16
#define DEFAULT_RUN_SECONDS 8
#define MAX_RUN_SECONDS 300

#define ESC_PERIOD_US 20000U
#define TCS_PERIOD_US 40000U
#define FUEL_PERIOD_US 80000U
#define DIAGNOSTICS_PERIOD_US 160000U

#define ESC_OFFSET_US 5000U
#define TCS_OFFSET_US 25000U
#define FUEL_OFFSET_US 45000U
#define DIAGNOSTICS_OFFSET_US 70000U

#define ESC_TIMER_SIGNAL (SIGRTMIN + 1)
#define TCS_TIMER_SIGNAL (SIGRTMIN + 2)

#define TASK_CLOCK CLOCK_MONOTONIC

#endif
