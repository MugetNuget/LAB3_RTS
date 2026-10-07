#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "config.h"

typedef enum {
    SIGNAL_BRAKE = 0,
    SIGNAL_DRIVE_TORQUE,
    SIGNAL_FUEL_RATE,
    SIGNAL_SPEED,
    SIGNAL_COUNT
} signal_channel_t;

typedef struct {
    uint64_t sequence;
    double time_seconds;
    float signals[SIGNAL_COUNT];
    bool stability_warning;
} vehicle_sample_t;

typedef struct {
    vehicle_sample_t items[LOG_CAPACITY];
    size_t read_index;
    size_t write_index;
    size_t size;
    pthread_mutex_t mutex;
    pthread_cond_t item_ready;
    pthread_cond_t space_ready;
} sample_queue_t;

typedef struct {
    uint64_t releases;
    int64_t shortest_period_us;
    int64_t longest_period_us;
    int64_t greatest_jitter_us;
    long double accumulated_period_us;
    struct timespec previous_release;
} release_stats_t;

typedef struct {
    float signals[SIGNAL_COUNT];
    float lateral_acceleration_g;
    float wheel_slip_percent;
    bool stability_warning;
    bool running;
    pthread_mutex_t mutex;
    sample_queue_t samples;
    release_stats_t stats[TASK_COUNT];
    struct timespec epoch;
} vehicle_t;

typedef struct {
    vehicle_t *vehicle;
    unsigned task_index;
    bool failed;
} task_context_t;

int vehicle_init(vehicle_t *vehicle);
void vehicle_destroy(vehicle_t *vehicle);
void vehicle_stop(vehicle_t *vehicle);
bool vehicle_is_running(vehicle_t *vehicle);

void record_release(vehicle_t *vehicle, unsigned task_index,
                    uint64_t nominal_period_us);
double elapsed_seconds(const struct timespec *epoch);

bool sample_queue_put(vehicle_t *vehicle, const vehicle_sample_t *sample);
bool sample_queue_take(vehicle_t *vehicle, vehicle_sample_t *sample,
                       bool wait_for_item);
void sample_queue_wake_all(vehicle_t *vehicle);

#endif
