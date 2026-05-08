#include "velocity_openloop.h"
#include "pwm_init.h"

#include <math.h>

#include "esp_timer.h"

float openloop_voltage = 0.0f;

float PID_Controller(float error, PIDController *controller)
{
    float Kp = controller->Kp;
    float Ki = controller->Ki;
    float Kd = controller->Kd;
    float limit = controller->limit;
    float ramp = controller->ramp;

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
    if (Ts <= 0 || Ts > 0.5f) {
        Ts = 1e-3f;
    }

    error_integral[idx] += error * Ts;
    if (Ki > 0) {
        error_integral[idx] = constrain(error_integral[idx], -limit / Ki, limit / Ki);
    } else {
        error_integral[idx] = 0;
    }

    float output = Kp * error + Ki * error_integral[idx] + Kd * (error - last_error[idx]) / Ts;
    output = constrain(output, -limit, limit);

    if (ramp > 0 && fabs(output - last_output[idx]) > ramp * Ts) {
        output = last_output[idx] + ((output > last_output[idx]) ? ramp * Ts : -ramp * Ts);
    }

    last_output[idx] = output;
    last_error[idx] = error;
    last_us[idx] = now_us;

    return output;
}
