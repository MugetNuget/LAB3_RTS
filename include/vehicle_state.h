#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "config.h"

/* Estructura para muestras del buffer de telemetria */
typedef struct {
    uint32_t    seq_num;
    double      timestamp_sec;
    float       datos[4];
    bool        alerta_activa;
} telemetry_entry_t;

/* Buffer circular sincronizado */
typedef struct {
    telemetry_entry_t   buffer[TELEMETRY_BUFFER_SIZE];
    int                 in;
    int                 out;
    int                 count;
    pthread_mutex_t     mutex;
    pthread_cond_t      not_full;
    pthread_cond_t      not_empty;
} circular_telemetry_t;

/* Metricas de determinismo por tarea */
typedef struct {
    uint32_t            activations;
    int64_t             min_period_us;
    int64_t             max_period_us;
    double              avg_period_us;
    int64_t             max_jitter_us;
    struct timespec     last_activation;
} task_metrics_t;

/* Estado compartido del vehiculo */
typedef struct {
    float               arreglo_compartido[4];
    float               velocidad_kmh;
    float               freno_esc;
    float               par_motor;
    float               inyeccion_combustible;
    bool                alerta_activa;
    bool                system_running;

    pthread_mutex_t     state_mutex;
    pthread_cond_t      cond_alert;

    circular_telemetry_t telemetry;

    task_metrics_t      metrics_tau1;
    task_metrics_t      metrics_tau2;
    task_metrics_t      metrics_tau3;
    task_metrics_t      metrics_tau4;
} vehicle_state_t;

extern vehicle_state_t g_vehicle_state;

void vehicle_state_init(vehicle_state_t *vs);
void vehicle_state_destroy(vehicle_state_t *vs);

void telemetry_push(circular_telemetry_t *ct, const telemetry_entry_t *entry);
bool telemetry_pop(circular_telemetry_t *ct, telemetry_entry_t *entry, bool blocking);

void update_task_metrics(task_metrics_t *metrics, uint64_t nominal_period_us);

#endif /* VEHICLE_STATE_H */
