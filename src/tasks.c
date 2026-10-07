#include "tasks.h"
#include "timers_clocks.h"

static float limit_range(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

static bool lock_vehicle(vehicle_t *vehicle)
{
    int error = pthread_mutex_lock(&vehicle->mutex);
    if (error != 0) {
        fprintf(stderr, "pthread_mutex_lock(tarea): %s\n", strerror(error));
        return false;
    }
    return true;
}

static bool unlock_vehicle(vehicle_t *vehicle)
{
    int error = pthread_mutex_unlock(&vehicle->mutex);
    if (error != 0) {
        fprintf(stderr, "pthread_mutex_unlock(tarea): %s\n", strerror(error));
        return false;
    }
    return true;
}

static bool update_stability(task_context_t *context)
{
    vehicle_t *vehicle = context->vehicle;
    double phase = fmod(elapsed_seconds(&vehicle->epoch), 5.0);
    float lateral_g = (phase >= 1.2 && phase < 2.4) ? 0.93f : 0.22f;
    const float stability_limit_g = 0.65f;
    float brake_request = lateral_g > stability_limit_g
        ? (lateral_g - stability_limit_g) * 120.0f
        : 0.0f;

    if (!lock_vehicle(vehicle)) {
        return false;
    }
    vehicle->lateral_acceleration_g = lateral_g;
    vehicle->stability_warning = lateral_g > stability_limit_g;
    vehicle->signals[SIGNAL_BRAKE] = limit_range(brake_request, 0.0f, 45.0f);
    return unlock_vehicle(vehicle);
}

static bool update_traction(task_context_t *context)
{
    vehicle_t *vehicle = context->vehicle;
    double phase = fmod(elapsed_seconds(&vehicle->epoch), 6.0);
    float estimated_slip = (phase >= 2.0 && phase < 3.3) ? 24.0f : 5.0f;
    vehicle_sample_t sample;

    if (!lock_vehicle(vehicle)) {
        return false;
    }
    float excess_slip = fmaxf(estimated_slip - 10.0f, 0.0f);
    float torque = 96.0f - excess_slip * 2.0f
        - vehicle->signals[SIGNAL_BRAKE] * 0.22f;
    vehicle->wheel_slip_percent = estimated_slip;
    vehicle->signals[SIGNAL_DRIVE_TORQUE] = limit_range(torque, 28.0f, 96.0f);

    sample.sequence = vehicle->stats[context->task_index].releases;
    sample.time_seconds = elapsed_seconds(&vehicle->epoch);
    memcpy(sample.signals, vehicle->signals, sizeof(sample.signals));
    sample.stability_warning = vehicle->stability_warning;
    if (!unlock_vehicle(vehicle)) {
        return false;
    }

    return sample_queue_put(vehicle, &sample)
        || !vehicle_is_running(vehicle);
}

static bool update_fuel_and_speed(task_context_t *context)
{
    vehicle_t *vehicle = context->vehicle;
    const float step_seconds = (float)FUEL_PERIOD_US / 1000000.0f;

    if (!lock_vehicle(vehicle)) {
        return false;
    }
    float brake = vehicle->signals[SIGNAL_BRAKE];
    float torque = vehicle->signals[SIGNAL_DRIVE_TORQUE];
    float speed = vehicle->signals[SIGNAL_SPEED];
    float fuel_rate = 24.0f + torque * 0.62f + speed * 0.08f - brake * 0.18f;
    float acceleration = torque * 0.022f - brake * 0.038f - 0.22f;

    if (vehicle->stability_warning) {
        fuel_rate *= 0.88f;
    }
    vehicle->signals[SIGNAL_FUEL_RATE] = limit_range(fuel_rate, 15.0f, 90.0f);
    vehicle->signals[SIGNAL_SPEED] =
        limit_range(speed + acceleration * step_seconds, 0.0f, 130.0f);
    return unlock_vehicle(vehicle);
}

static bool report_samples(task_context_t *context)
{
    vehicle_t *vehicle = context->vehicle;
    vehicle_sample_t sample;
    size_t collected = 0;

    if (!sample_queue_take(vehicle, &sample, true)) {
        return !vehicle_is_running(vehicle);
    }
    ++collected;
    vehicle_sample_t latest = sample;
    while (sample_queue_take(vehicle, &sample, false)) {
        latest = sample;
        ++collected;
    }

    printf("[diag %5.2f s] muestras=%zu seq=%llu | freno=%4.1f%% "
           "par=%4.1f%% combustible=%4.1f%% velocidad=%5.1f km/h | estabilidad=%s\n",
           elapsed_seconds(&vehicle->epoch), collected,
           (unsigned long long)latest.sequence,
           latest.signals[SIGNAL_BRAKE],
           latest.signals[SIGNAL_DRIVE_TORQUE],
           latest.signals[SIGNAL_FUEL_RATE],
           latest.signals[SIGNAL_SPEED],
           latest.stability_warning ? "alerta" : "normal");
    fflush(stdout);
    return true;
}

void *task_stability(void *argument)
{
    task_context_t *context = argument;
    vehicle_t *vehicle = context->vehicle;
    signal_timer_t timer;

    if (signal_timer_start(&timer, &vehicle->epoch, ESC_OFFSET_US,
                           ESC_PERIOD_US, ESC_TIMER_SIGNAL) != 0) {
        context->failed = true;
        return NULL;
    }
    while (vehicle_is_running(vehicle)) {
        if (signal_timer_wait(&timer) != 0) {
            context->failed = true;
            break;
        }
        if (!vehicle_is_running(vehicle)) {
            break;
        }
        record_release(vehicle, context->task_index, ESC_PERIOD_US);
        if (!update_stability(context)) {
            context->failed = true;
            break;
        }
    }
    if (signal_timer_stop(&timer) != 0) {
        context->failed = true;
    }
    return NULL;
}

void *task_traction(void *argument)
{
    task_context_t *context = argument;
    vehicle_t *vehicle = context->vehicle;
    signal_timer_t timer;

    if (signal_timer_start(&timer, &vehicle->epoch, TCS_OFFSET_US,
                           TCS_PERIOD_US, TCS_TIMER_SIGNAL) != 0) {
        context->failed = true;
        return NULL;
    }
    while (vehicle_is_running(vehicle)) {
        if (signal_timer_wait(&timer) != 0) {
            context->failed = true;
            break;
        }
        if (!vehicle_is_running(vehicle)) {
            break;
        }
        record_release(vehicle, context->task_index, TCS_PERIOD_US);
        if (!update_traction(context)) {
            context->failed = true;
            break;
        }
    }
    if (signal_timer_stop(&timer) != 0) {
        context->failed = true;
    }
    return NULL;
}

void *task_fuel(void *argument)
{
    task_context_t *context = argument;
    vehicle_t *vehicle = context->vehicle;
    clock_schedule_t schedule;

    if (clock_schedule_start(&schedule, &vehicle->epoch, FUEL_OFFSET_US,
                             FUEL_PERIOD_US) != 0) {
        context->failed = true;
        return NULL;
    }
    while (vehicle_is_running(vehicle)) {
        if (clock_schedule_wait(&schedule) != 0) {
            context->failed = true;
            break;
        }
        if (!vehicle_is_running(vehicle)) {
            break;
        }
        record_release(vehicle, context->task_index, FUEL_PERIOD_US);
        if (!update_fuel_and_speed(context)) {
            context->failed = true;
            break;
        }
    }
    return NULL;
}

void *task_diagnostics(void *argument)
{
    task_context_t *context = argument;
    vehicle_t *vehicle = context->vehicle;
    clock_schedule_t schedule;

    if (clock_schedule_start(&schedule, &vehicle->epoch, DIAGNOSTICS_OFFSET_US,
                             DIAGNOSTICS_PERIOD_US) != 0) {
        context->failed = true;
        return NULL;
    }
    while (vehicle_is_running(vehicle)) {
        if (clock_schedule_wait(&schedule) != 0) {
            context->failed = true;
            break;
        }
        if (!vehicle_is_running(vehicle)) {
            break;
        }
        record_release(vehicle, context->task_index, DIAGNOSTICS_PERIOD_US);
        if (!report_samples(context)) {
            if (vehicle_is_running(vehicle)) {
                context->failed = true;
            }
            break;
        }
    }
    return NULL;
}
