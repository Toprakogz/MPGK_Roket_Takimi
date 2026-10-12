// Write your filters in this file.
// Keep every piece of filter state here and reset all of it in filter_init().
#include "filter.h"
#include "cmath"
static bool is_initialized = false;
static uint32_t sample_count = 0;
static float ref_pressure_pa = 101325.0f;
static float pressure_sum = 0.0f;
static float estimated_altitude_m = 0.0f;
static float estimated_speed_mps = 0.0f;
static float last_altitude_m = 0.0f;
static uint32_t last_t_ms = 0;

void filter_init(void) {
    // TODO: reset all filter state
    is_initialized = false;
    sample_count = 0;
    ref_pressure_pa = 101325.0f;
    pressure_sum = 0.0f;
    estimated_altitude_m = 0.0f;
    estimated_speed_mps = 0.0f;
    last_altitude_m = 0.0f;
    last_t_ms = 0;
}

void filter_step(const SensorSample *in, FilterOutput *out) {
    (void)in;
    // TODO: estimate altitude above the pad, vertical speed and vertical acceleration
    float dt = 0.01f;
    if (last_t_ms != 0 && in->t_ms > last_t_ms) {
        dt = (float)(in->t_ms - last_t_ms) / 1000.0f;
    }
    last_t_ms = in->t_ms;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
    if (!is_initialized) {
        if (in->t_ms < 25000.0f) {
            pressure_sum += in->pressure_pa;
            sample_count++;
            ref_pressure_pa = pressure_sum / (float)sample_count;
        } else {
            is_initialized = true;
        }
    }
    float baro_altitude = 0.0f;
    if (in->pressure_pa > 0.0f && ref_pressure_pa > 0.0f) {
        baro_altitude = 44330.0f * (1.0f - powf(in->pressure_pa / ref_pressure_pa, 0.1903f));
    }
    if (sample_count > 100 && fabsf(baro_altitude - last_altitude_m) > 40.0f) {
        baro_altitude = last_altitude_m;
    }
    float raw_vertical_accel = (in->acc_g[2] - 1.0f) * 9.81f;

    float predicted_altitude = estimated_altitude_m + estimated_speed_mps * dt + 0.5f * raw_vertical_accel * dt * dt;
    float predicted_speed = estimated_speed_mps + raw_vertical_accel * dt;

    float alpha = 0.92f;
    estimated_altitude_m = alpha * predicted_altitude + (1.0f - alpha) * baro_altitude;

    float baro_speed = (baro_altitude - last_altitude_m) / dt;
    estimated_speed_mps = predicted_speed * 0.8f + baro_speed * 0.2f;

    last_altitude_m = baro_altitude;
    out->altitude_m = estimated_altitude_m;
    out->vertical_speed_mps = estimated_speed_mps;
    out->vertical_accel_mps2 = raw_vertical_accel;
}
