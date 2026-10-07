#include "vehicle_state.h"
#include "timers_clocks.h"

vehicle_state_t g_vehicle_state;

void vehicle_state_init(vehicle_state_t *vs)
{
    memset(vs, 0, sizeof(vehicle_state_t));

    vs->velocidad_kmh         = 80.0f;
    vs->freno_esc             = 0.0f;
    vs->par_motor             = 100.0f;
    vs->inyeccion_combustible = 80.0f;
    vs->alerta_activa         = false;
    vs->system_running        = true;

    vs->arreglo_compartido[0] = vs->freno_esc;
    vs->arreglo_compartido[1] = vs->par_motor;
    vs->arreglo_compartido[2] = vs->inyeccion_combustible;
    vs->arreglo_compartido[3] = vs->velocidad_kmh;

    pthread_mutex_init(&vs->state_mutex, NULL);
    pthread_cond_init(&vs->cond_alert, NULL);

    vs->telemetry.in    = 0;
    vs->telemetry.out   = 0;
    vs->telemetry.count = 0;
    pthread_mutex_init(&vs->telemetry.mutex, NULL);
    pthread_cond_init(&vs->telemetry.not_full, NULL);
    pthread_cond_init(&vs->telemetry.not_empty, NULL);

    task_metrics_t *metrics_array[4] = {
        &vs->metrics_tau1, &vs->metrics_tau2, &vs->metrics_tau3, &vs->metrics_tau4
    };

    for (int i = 0; i < 4; i++) {
        metrics_array[i]->activations   = 0;
        metrics_array[i]->min_period_us = INT64_MAX;
        metrics_array[i]->max_period_us = 0;
        metrics_array[i]->avg_period_us = 0.0;
        metrics_array[i]->max_jitter_us = 0;
        clock_gettime(RT_CLOCK_SOURCE, &metrics_array[i]->last_activation);
    }
}

void vehicle_state_destroy(vehicle_state_t *vs)
{
    pthread_mutex_destroy(&vs->state_mutex);
    pthread_cond_destroy(&vs->cond_alert);

    pthread_mutex_destroy(&vs->telemetry.mutex);
    pthread_cond_destroy(&vs->telemetry.not_full);
    pthread_cond_destroy(&vs->telemetry.not_empty);
}

void telemetry_push(circular_telemetry_t *ct, const telemetry_entry_t *entry)
{
    pthread_mutex_lock(&ct->mutex);

    while (ct->count == TELEMETRY_BUFFER_SIZE && g_vehicle_state.system_running) {
        pthread_cond_wait(&ct->not_full, &ct->mutex);
    }

    if (!g_vehicle_state.system_running) {
        pthread_mutex_unlock(&ct->mutex);
        return;
    }

    ct->buffer[ct->in] = *entry;
    ct->in = (ct->in + 1) % TELEMETRY_BUFFER_SIZE;
    ct->count++;

    pthread_cond_signal(&ct->not_empty);
    pthread_mutex_unlock(&ct->mutex);
}

bool telemetry_pop(circular_telemetry_t *ct, telemetry_entry_t *entry, bool blocking)
{
    pthread_mutex_lock(&ct->mutex);

    if (blocking) {
        while (ct->count == 0 && g_vehicle_state.system_running) {
            pthread_cond_wait(&ct->not_empty, &ct->mutex);
        }
    }

    if (ct->count == 0) {
        pthread_mutex_unlock(&ct->mutex);
        return false;
    }

    *entry = ct->buffer[ct->out];
    ct->out = (ct->out + 1) % TELEMETRY_BUFFER_SIZE;
    ct->count--;

    pthread_cond_signal(&ct->not_full);
    pthread_mutex_unlock(&ct->mutex);
    return true;
}

void update_task_metrics(task_metrics_t *metrics, uint64_t nominal_period_us)
{
    struct timespec now;
    clock_gettime(RT_CLOCK_SOURCE, &now);

    if (metrics->activations > 0) {
        int64_t period_us = timespec_diff_us(&metrics->last_activation, &now);
        int64_t jitter_us = llabs(period_us - (int64_t)nominal_period_us);

        if (period_us < metrics->min_period_us) metrics->min_period_us = period_us;
        if (period_us > metrics->max_period_us) metrics->max_period_us = period_us;
        if (jitter_us > metrics->max_jitter_us) metrics->max_jitter_us = jitter_us;

        metrics->avg_period_us = ((metrics->avg_period_us * (metrics->activations - 1)) + period_us) 
                                 / (double)metrics->activations;
    }

    metrics->last_activation = now;
    metrics->activations++;
}
