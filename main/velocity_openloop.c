#include "velocity_openloop.h"
#include "pwm_init.h"

#include <math.h>
#include "esp_timer.h"

float openloop_voltage = 0.0f;

float velocityOpenloop(float target_v)
{
    static int64_t last_us = 0;
    static float sum_angle = 0.0f;

    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - last_us) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    sum_angle = normalizeAngle(sum_angle + target_v * Ts);
    setPhaseVoltage(openloop_voltage, 0.0f, electricalAngle(sum_angle));

    last_us = now_us;
    return 0.0f;
}

float PID_Controller(float error, PIDController *controller)
{
    if (!controller) return 0.0f;

    static int64_t last_us[PID_MAX_COUNT] = {0};
    static float last_error[PID_MAX_COUNT] = {0.0f};
    static float error_integral[PID_MAX_COUNT] = {0.0f};
    static float last_output[PID_MAX_COUNT] = {0.0f};

    int idx = 0;
    for (int i = 0; i < PID_MAX_COUNT; i++) {
        if (controller == &pid[i]) {
            idx = i;
            break;
        }
    }

    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - last_us[idx]) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    if (controller->Ki > 0.0f) {
        error_integral[idx] += error * Ts;
        error_integral[idx] = constrain(error_integral[idx], -controller->limit / controller->Ki, controller->limit / controller->Ki);
    } else {
        error_integral[idx] = 0.0f;
    }

    float output = controller->Kp * error
        + controller->Ki * error_integral[idx]
        + controller->Kd * (error - last_error[idx]) / Ts;
    output = constrain(output, -controller->limit, controller->limit);

    if (controller->ramp > 0.0f && fabsf(output - last_output[idx]) > controller->ramp * Ts) {
        output = last_output[idx] + ((output > last_output[idx]) ? controller->ramp * Ts : -controller->ramp * Ts);
    }

    last_output[idx] = output;
    last_error[idx] = error;
    last_us[idx] = now_us;

    return output;
}
