#include "tasks.h"
#include "timers_clocks.h"

/* Job Body Tau 1: Control de Estabilidad (ESC) - 20ms */
void job_body_tau1_esc(void)
{
    update_task_metrics(&g_vehicle_state.metrics_tau1, PERIOD_TAU1_US);

    double t = get_elapsed_time_sec();

    float inestabilidad = 0.0f;
    if (t >= 3.0 && t <= 6.0) {
        inestabilidad = 75.0f;
    }

    pthread_mutex_lock(&g_vehicle_state.state_mutex);

    if (inestabilidad > 50.0f) {
        g_vehicle_state.freno_esc = inestabilidad * 0.8f;
        g_vehicle_state.alerta_activa = true;
        pthread_cond_broadcast(&g_vehicle_state.cond_alert);
    } else {
        g_vehicle_state.freno_esc = 0.0f;
        g_vehicle_state.alerta_activa = false;
    }

    g_vehicle_state.arreglo_compartido[0] = g_vehicle_state.freno_esc;

    pthread_mutex_unlock(&g_vehicle_state.state_mutex);
}

/* Job Body Tau 2: Control de Traccion (TCS) - 40ms */
void job_body_tau2_tcs(void)
{
    update_task_metrics(&g_vehicle_state.metrics_tau2, PERIOD_TAU2_US);

    double t = get_elapsed_time_sec();

    float patinamiento = 0.0f;
    if (t >= 4.0 && t <= 7.0) {
        patinamiento = 40.0f;
    }

    pthread_mutex_lock(&g_vehicle_state.state_mutex);

    float par = 100.0f - patinamiento - (g_vehicle_state.freno_esc * 0.5f);
    if (par < 20.0f) par = 20.0f;
    g_vehicle_state.par_motor = par;

    g_vehicle_state.arreglo_compartido[1] = g_vehicle_state.par_motor;

    static uint32_t s_seq = 0;
    telemetry_entry_t entry;
    entry.seq_num       = ++s_seq;
    entry.timestamp_sec = t;
    for (int i = 0; i < 4; i++) {
        entry.datos[i] = g_vehicle_state.arreglo_compartido[i];
    }
    entry.alerta_activa = g_vehicle_state.alerta_activa;

    pthread_mutex_unlock(&g_vehicle_state.state_mutex);

    telemetry_push(&g_vehicle_state.telemetry, &entry);
}

/* Job Body Tau 3: Inyeccion de Combustible - 80ms */
void job_body_tau3_injection(void)
{
    update_task_metrics(&g_vehicle_state.metrics_tau3, PERIOD_TAU3_US);

    pthread_mutex_lock(&g_vehicle_state.state_mutex);

    float iny = g_vehicle_state.par_motor * 0.8f;
    if (g_vehicle_state.alerta_activa) {
        iny *= 0.5f;
    }
    g_vehicle_state.inyeccion_combustible = iny;

    g_vehicle_state.arreglo_compartido[2] = g_vehicle_state.inyeccion_combustible;

    pthread_mutex_unlock(&g_vehicle_state.state_mutex);
}

/* Job Body Tau 4: Telemetria y Diagnostico - 160ms */
void job_body_tau4_telemetry(void)
{
    update_task_metrics(&g_vehicle_state.metrics_tau4, PERIOD_TAU4_US);

    telemetry_entry_t entry;
    int samples_processed = 0;
    while (telemetry_pop(&g_vehicle_state.telemetry, &entry, false)) {
        samples_processed++;
    }

    pthread_mutex_lock(&g_vehicle_state.state_mutex);

    double t = get_elapsed_time_sec();
    g_vehicle_state.arreglo_compartido[3] = g_vehicle_state.velocidad_kmh;

    printf("[%6.3f s] Arreglo:[ESC:%3.0f%% | TCS(Par):%3.0f%% | Iny:%3.0f%% | Vel:%3.0f km/h] | Alerta: %-8s | Buffer: %d\n",
           t,
           g_vehicle_state.arreglo_compartido[0],
           g_vehicle_state.arreglo_compartido[1],
           g_vehicle_state.arreglo_compartido[2],
           g_vehicle_state.arreglo_compartido[3],
           g_vehicle_state.alerta_activa ? "¡PELIGRO!" : "OK",
           samples_processed);

    pthread_mutex_unlock(&g_vehicle_state.state_mutex);
}

void* thread_tau1_esc(void *arg)
{
    (void)arg;
    posix_timer_task_t *timer = posix_timer_create_and_arm(OFFSET_TAU1_US, PERIOD_TAU1_US, SIG_TAU1_ESC);
    if (!timer) {
        fprintf(stderr, "Error inicializando timer Tau 1\n");
        return NULL;
    }

    while (g_vehicle_state.system_running) {
        posix_timer_wait_activation(timer);
        if (!g_vehicle_state.system_running) break;
        job_body_tau1_esc();
    }

    posix_timer_destroy(timer);
    return NULL;
}

void* thread_tau2_tcs(void *arg)
{
    (void)arg;
    posix_timer_task_t *timer = posix_timer_create_and_arm(OFFSET_TAU2_US, PERIOD_TAU2_US, SIG_TAU2_TCS);
    if (!timer) {
        fprintf(stderr, "Error inicializando timer Tau 2\n");
        return NULL;
    }

    while (g_vehicle_state.system_running) {
        posix_timer_wait_activation(timer);
        if (!g_vehicle_state.system_running) break;
        job_body_tau2_tcs();
    }

    posix_timer_destroy(timer);
    return NULL;
}

void* thread_tau3_injection(void *arg)
{
    (void)arg;
    posix_clock_task_t *clock_task = posix_clock_init_task(OFFSET_TAU3_US, PERIOD_TAU3_US, RT_CLOCK_SOURCE);
    if (!clock_task) {
        fprintf(stderr, "Error inicializando clock Tau 3\n");
        return NULL;
    }

    while (g_vehicle_state.system_running) {
        posix_clock_wait_activation(clock_task);
        if (!g_vehicle_state.system_running) break;
        job_body_tau3_injection();
    }

    posix_clock_destroy(clock_task);
    return NULL;
}

void* thread_tau4_telemetry(void *arg)
{
    (void)arg;
    posix_clock_task_t *clock_task = posix_clock_init_task(OFFSET_TAU4_US, PERIOD_TAU4_US, RT_CLOCK_SOURCE);
    if (!clock_task) {
        fprintf(stderr, "Error inicializando clock Tau 4\n");
        return NULL;
    }

    while (g_vehicle_state.system_running) {
        posix_clock_wait_activation(clock_task);
        if (!g_vehicle_state.system_running) break;
        job_body_tau4_telemetry();
    }

    posix_clock_destroy(clock_task);
    return NULL;
}
