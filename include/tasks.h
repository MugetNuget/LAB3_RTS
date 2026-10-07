#ifndef TASKS_H
#define TASKS_H

#include "config.h"
#include "vehicle_state.h"

/* Hilos periodicos */
void* thread_tau1_esc(void *arg);
void* thread_tau2_tcs(void *arg);
void* thread_tau3_injection(void *arg);
void* thread_tau4_telemetry(void *arg);

/* Job Bodies */
void job_body_tau1_esc(void);
void job_body_tau2_tcs(void);
void job_body_tau3_injection(void);
void job_body_tau4_telemetry(void);

#endif /* TASKS_H */
