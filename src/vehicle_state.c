#include "vehicle_state.h"
#include "timers_clocks.h"

static void report_pthread_error(const char *operation, int error)
{
    fprintf(stderr, "%s: %s\n", operation, strerror(error));
}

int vehicle_init(vehicle_t *vehicle)
{
    memset(vehicle, 0, sizeof(*vehicle));
    vehicle->signals[SIGNAL_DRIVE_TORQUE] = 72.0f;
    vehicle->signals[SIGNAL_FUEL_RATE] = 58.0f;
    vehicle->signals[SIGNAL_SPEED] = 54.0f;
    vehicle->running = true;

    int error = pthread_mutex_init(&vehicle->mutex, NULL);
    if (error != 0) {
        report_pthread_error("pthread_mutex_init(vehicle)", error);
        return error;
    }

    error = pthread_mutex_init(&vehicle->samples.mutex, NULL);
    if (error != 0) {
        report_pthread_error("pthread_mutex_init(samples)", error);
        pthread_mutex_destroy(&vehicle->mutex);
        return error;
    }

    error = pthread_cond_init(&vehicle->samples.item_ready, NULL);
    if (error != 0) {
        report_pthread_error("pthread_cond_init(item_ready)", error);
        pthread_mutex_destroy(&vehicle->samples.mutex);
        pthread_mutex_destroy(&vehicle->mutex);
        return error;
    }

    error = pthread_cond_init(&vehicle->samples.space_ready, NULL);
    if (error != 0) {
        report_pthread_error("pthread_cond_init(space_ready)", error);
        pthread_cond_destroy(&vehicle->samples.item_ready);
        pthread_mutex_destroy(&vehicle->samples.mutex);
        pthread_mutex_destroy(&vehicle->mutex);
        return error;
    }

    for (unsigned i = 0; i < TASK_COUNT; ++i) {
        vehicle->stats[i].shortest_period_us = INT64_MAX;
    }
    return 0;
}

void vehicle_destroy(vehicle_t *vehicle)
{
    pthread_cond_destroy(&vehicle->samples.space_ready);
    pthread_cond_destroy(&vehicle->samples.item_ready);
    pthread_mutex_destroy(&vehicle->samples.mutex);
    pthread_mutex_destroy(&vehicle->mutex);
}

void vehicle_stop(vehicle_t *vehicle)
{
    int error = pthread_mutex_lock(&vehicle->mutex);
    if (error != 0) {
        report_pthread_error("pthread_mutex_lock(vehicle stop)", error);
        return;
    }
    vehicle->running = false;
    pthread_mutex_unlock(&vehicle->mutex);
    sample_queue_wake_all(vehicle);
}

bool vehicle_is_running(vehicle_t *vehicle)
{
    bool running;
    int error = pthread_mutex_lock(&vehicle->mutex);
    if (error != 0) {
        report_pthread_error("pthread_mutex_lock(vehicle status)", error);
        return false;
    }
    running = vehicle->running;
    pthread_mutex_unlock(&vehicle->mutex);
    return running;
}

double elapsed_seconds(const struct timespec *epoch)
{
    struct timespec now;
    if (clock_gettime(TASK_CLOCK, &now) != 0) {
        perror("clock_gettime");
        return 0.0;
    }

    return (double)timespec_difference_microseconds(epoch, &now) / 1000000.0;
}

void record_release(vehicle_t *vehicle, unsigned task_index,
                    uint64_t nominal_period_us)
{
    struct timespec now;
    if (task_index >= TASK_COUNT) {
        fprintf(stderr, "Indice de tarea fuera de rango: %u\n", task_index);
        return;
    }
    if (clock_gettime(TASK_CLOCK, &now) != 0) {
        perror("clock_gettime al medir activacion");
        return;
    }

    release_stats_t *stats = &vehicle->stats[task_index];
    if (stats->releases > 0) {
        int64_t measured = timespec_difference_microseconds(&stats->previous_release, &now);
        int64_t deviation = llabs(measured - (int64_t)nominal_period_us);
        if (measured < stats->shortest_period_us) {
            stats->shortest_period_us = measured;
        }
        if (measured > stats->longest_period_us) {
            stats->longest_period_us = measured;
        }
        if (deviation > stats->greatest_jitter_us) {
            stats->greatest_jitter_us = deviation;
        }
        stats->accumulated_period_us += measured;
    }

    stats->previous_release = now;
    ++stats->releases;
}

bool sample_queue_put(vehicle_t *vehicle, const vehicle_sample_t *sample)
{
    sample_queue_t *queue = &vehicle->samples;
    int error = pthread_mutex_lock(&queue->mutex);
    if (error != 0) {
        report_pthread_error("pthread_mutex_lock(sample producer)", error);
        return false;
    }

    while (queue->size == LOG_CAPACITY && vehicle_is_running(vehicle)) {
        error = pthread_cond_wait(&queue->space_ready, &queue->mutex);
        if (error != 0) {
            report_pthread_error("pthread_cond_wait(space_ready)", error);
            pthread_mutex_unlock(&queue->mutex);
            return false;
        }
    }

    if (!vehicle_is_running(vehicle)) {
        pthread_mutex_unlock(&queue->mutex);
        return false;
    }

    queue->items[queue->write_index] = *sample;
    queue->write_index = (queue->write_index + 1U) % LOG_CAPACITY;
    ++queue->size;

    error = pthread_cond_signal(&queue->item_ready);
    if (error != 0) {
        report_pthread_error("pthread_cond_signal(item_ready)", error);
    }
    pthread_mutex_unlock(&queue->mutex);
    return error == 0;
}

bool sample_queue_take(vehicle_t *vehicle, vehicle_sample_t *sample,
                       bool wait_for_item)
{
    sample_queue_t *queue = &vehicle->samples;
    int error = pthread_mutex_lock(&queue->mutex);
    if (error != 0) {
        report_pthread_error("pthread_mutex_lock(sample consumer)", error);
        return false;
    }

    while (wait_for_item && queue->size == 0 && vehicle_is_running(vehicle)) {
        error = pthread_cond_wait(&queue->item_ready, &queue->mutex);
        if (error != 0) {
            report_pthread_error("pthread_cond_wait(item_ready)", error);
            pthread_mutex_unlock(&queue->mutex);
            return false;
        }
    }

    if (queue->size == 0) {
        pthread_mutex_unlock(&queue->mutex);
        return false;
    }

    *sample = queue->items[queue->read_index];
    queue->read_index = (queue->read_index + 1U) % LOG_CAPACITY;
    --queue->size;

    error = pthread_cond_signal(&queue->space_ready);
    if (error != 0) {
        report_pthread_error("pthread_cond_signal(space_ready)", error);
    }
    pthread_mutex_unlock(&queue->mutex);
    return error == 0;
}

void sample_queue_wake_all(vehicle_t *vehicle)
{
    sample_queue_t *queue = &vehicle->samples;
    int error = pthread_mutex_lock(&queue->mutex);
    if (error != 0) {
        report_pthread_error("pthread_mutex_lock(queue shutdown)", error);
        return;
    }
    pthread_cond_broadcast(&queue->item_ready);
    pthread_cond_broadcast(&queue->space_ready);
    pthread_mutex_unlock(&queue->mutex);
}
