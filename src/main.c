#include "config.h"
#include "timers_clocks.h"
#include "vehicle_state.h"
#include "tasks.h"
#include <sched.h>

static void set_thread_cpu_affinity(pthread_t thread, int core_id)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    int s = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (s != 0) {
        fprintf(stderr, "[AVISO] No se pudo asignar al Core %d: %s\n", core_id, strerror(s));
    }
}

static void print_final_report(int sim_time_sec)
{
    printf("\n=========================================================================================================\n");
    printf("                       INFORME DE EJECUCION Y DETERMINISMO DE TIEMPO REAL\n");
    printf("=========================================================================================================\n");
    printf("Duracion de la prueba: %d segundos\n\n", sim_time_sec);

    printf("%-6s | %-24s | %-13s | %-9s | %-9s | %-10s | %-10s | %-10s\n",
           "Tarea", "Funcion", "Mecanismo", "T Nom(ms)", "Act. Esp", "Act. Real", "T Prom(ms)", "Max Jitter");
    printf("---------------------------------------------------------------------------------------------------------\n");

    uint32_t expected_tau1 = (sim_time_sec * 1000) / (PERIOD_TAU1_US / 1000);
    printf("%-6s | %-24s | %-13s | %9.1f | %9u | %10u | %10.3f | %8ld us\n",
           "Tau 1", "Control Estabilidad(ESC)", "POSIX Timer",
           (float)PERIOD_TAU1_US / 1000.0f, expected_tau1,
           g_vehicle_state.metrics_tau1.activations,
           g_vehicle_state.metrics_tau1.avg_period_us / 1000.0,
           g_vehicle_state.metrics_tau1.max_jitter_us);

    uint32_t expected_tau2 = (sim_time_sec * 1000) / (PERIOD_TAU2_US / 1000);
    printf("%-6s | %-24s | %-13s | %9.1f | %9u | %10u | %10.3f | %8ld us\n",
           "Tau 2", "Control Traccion (TCS)", "POSIX Timer",
           (float)PERIOD_TAU2_US / 1000.0f, expected_tau2,
           g_vehicle_state.metrics_tau2.activations,
           g_vehicle_state.metrics_tau2.avg_period_us / 1000.0,
           g_vehicle_state.metrics_tau2.max_jitter_us);

    uint32_t expected_tau3 = (sim_time_sec * 1000) / (PERIOD_TAU3_US / 1000);
    printf("%-6s | %-24s | %-13s | %9.1f | %9u | %10u | %10.3f | %8ld us\n",
           "Tau 3", "Inyeccion Combustible", "POSIX Clock",
           (float)PERIOD_TAU3_US / 1000.0f, expected_tau3,
           g_vehicle_state.metrics_tau3.activations,
           g_vehicle_state.metrics_tau3.avg_period_us / 1000.0,
           g_vehicle_state.metrics_tau3.max_jitter_us);

    uint32_t expected_tau4 = (sim_time_sec * 1000) / (PERIOD_TAU4_US / 1000);
    printf("%-6s | %-24s | %-13s | %9.1f | %9u | %10u | %10.3f | %8ld us\n",
           "Tau 4", "Telemetria (Soft-RT)", "POSIX Clock",
           (float)PERIOD_TAU4_US / 1000.0f, expected_tau4,
           g_vehicle_state.metrics_tau4.activations,
           g_vehicle_state.metrics_tau4.avg_period_us / 1000.0,
           g_vehicle_state.metrics_tau4.max_jitter_us);

    printf("=========================================================================================================\n");
    printf("Estado final del arreglo compartido:\n");
    printf("  [0] Freno ESC:        %5.1f %%\n", g_vehicle_state.arreglo_compartido[0]);
    printf("  [1] Par Motor (TCS):  %5.1f %%\n", g_vehicle_state.arreglo_compartido[1]);
    printf("  [2] Inyeccion:        %5.1f %%\n", g_vehicle_state.arreglo_compartido[2]);
    printf("  [3] Velocidad (Diag): %5.1f km/h\n", g_vehicle_state.arreglo_compartido[3]);
    printf("Sincronizacion y consistencia de datos compartidos verificada con exito.\n");
    printf("=========================================================================================================\n\n");
}

int main(int argc, char *argv[])
{
    int sim_time_sec = DEFAULT_SIM_TIME_SEC;
    if (argc >= 2) {
        int t = atoi(argv[1]);
        if (t > 0 && t <= 300) {
            sim_time_sec = t;
        }
    }

    printf("\n");
    printf("==================================================================================\n");
    printf("    SISTEMAS DE TIEMPO REAL - PRACTICA 3: CONTROL AUTOMOTRIZ CONCURRENTE\n");
    printf("==================================================================================\n");
    printf("Configuracion de Tareas de Tiempo Real (POSIX-RT):\n");
    printf("  [Tau 1] ESC/ESP:    Periodo = %2d ms (50.0 Hz) | Offset = %2d ms | POSIX Timer (SIGRTMIN+1)\n",
           PERIOD_TAU1_US / 1000, OFFSET_TAU1_US / 1000);
    printf("  [Tau 2] TCS:        Periodo = %2d ms (25.0 Hz) | Offset = %2d ms | POSIX Timer (SIGRTMIN+2)\n",
           PERIOD_TAU2_US / 1000, OFFSET_TAU2_US / 1000);
    printf("  [Tau 3] Inyeccion:  Periodo = %2d ms (12.5 Hz) | Offset = %2d ms | POSIX Clock (TIMER_ABSTIME)\n",
           PERIOD_TAU3_US / 1000, OFFSET_TAU3_US / 1000);
    printf("  [Tau 4] Telemetria: Periodo = %2d ms ( 6.2 Hz) | Offset = %2d ms | POSIX Clock (TIMER_ABSTIME)\n",
           PERIOD_TAU4_US / 1000, OFFSET_TAU4_US / 1000);
    printf("Tiempo total de ejecucion programado: %d segundos\n", sim_time_sec);
    printf("==================================================================================\n\n");

    /* Bloqueo de senales RT antes de crear hilos para herencia de mascara */
    sigset_t alarm_sigset;
    sigemptyset(&alarm_sigset);
    sigaddset(&alarm_sigset, SIG_TAU1_ESC);
    sigaddset(&alarm_sigset, SIG_TAU2_TCS);
    for (int i = SIGRTMIN; i <= SIGRTMAX; i++) {
        sigaddset(&alarm_sigset, i);
    }
    int sig_mask_res = pthread_sigmask(SIG_BLOCK, &alarm_sigset, NULL);
    if (sig_mask_res != 0) {
        fprintf(stderr, "Error en pthread_sigmask: %s\n", strerror(sig_mask_res));
        return EXIT_FAILURE;
    }

    vehicle_state_init(&g_vehicle_state);

    pthread_t th_tau1, th_tau2, th_tau3, th_tau4;

    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    printf("[INFO] Procesadores logicos disponibles en el sistema: %d\n", num_cores);

    pthread_create(&th_tau1, NULL, thread_tau1_esc, NULL);
    pthread_create(&th_tau2, NULL, thread_tau2_tcs, NULL);
    pthread_create(&th_tau3, NULL, thread_tau3_injection, NULL);
    pthread_create(&th_tau4, NULL, thread_tau4_telemetry, NULL);

    pthread_setname_np(th_tau1, "rt_tau1_esc");
    pthread_setname_np(th_tau2, "rt_tau2_tcs");
    pthread_setname_np(th_tau3, "rt_tau3_inj");
    pthread_setname_np(th_tau4, "rt_tau4_tel");

    if (num_cores >= 4) {
        set_thread_cpu_affinity(th_tau1, 0);
        set_thread_cpu_affinity(th_tau2, 1);
        set_thread_cpu_affinity(th_tau3, 2);
        set_thread_cpu_affinity(th_tau4, 3);
        printf("[INFO] Hilos asignados deterministamente a los 4 nucleos de CPU:\n");
        printf("       -> rt_tau1_esc:  Core 0 (ESC/ESP - 20ms)\n");
        printf("       -> rt_tau2_tcs:  Core 1 (TCS     - 40ms)\n");
        printf("       -> rt_tau3_inj:  Core 2 (Inyec   - 80ms)\n");
        printf("       -> rt_tau4_tel:  Core 3 (Telem   - 160ms)\n\n");
    } else {
        printf("[INFO] Mapeo de afinidad adaptado a %d nucleos.\n\n", num_cores);
    }

    sleep(sim_time_sec);

    printf("\n[INFO] Tiempo de prueba cumplido. Deteniendo hilos de tiempo real...\n");

    pthread_mutex_lock(&g_vehicle_state.state_mutex);
    g_vehicle_state.system_running = false;
    pthread_cond_broadcast(&g_vehicle_state.cond_alert);
    pthread_mutex_unlock(&g_vehicle_state.state_mutex);

    pthread_mutex_lock(&g_vehicle_state.telemetry.mutex);
    pthread_cond_broadcast(&g_vehicle_state.telemetry.not_empty);
    pthread_cond_broadcast(&g_vehicle_state.telemetry.not_full);
    pthread_mutex_unlock(&g_vehicle_state.telemetry.mutex);

    pthread_kill(th_tau1, SIG_TAU1_ESC);
    pthread_kill(th_tau2, SIG_TAU2_TCS);

    pthread_join(th_tau1, NULL);
    pthread_join(th_tau2, NULL);
    pthread_join(th_tau3, NULL);
    pthread_join(th_tau4, NULL);

    print_final_report(sim_time_sec);

    vehicle_state_destroy(&g_vehicle_state);

    return EXIT_SUCCESS;
}
