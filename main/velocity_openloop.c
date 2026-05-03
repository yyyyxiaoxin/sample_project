#include "velocity_openloop.h"
#include "pwm_init.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"


float openloop_voltage = 0.0;

// 设置电压力矩环

//开环速度
float velocityOpenloop(float target_v){
    static int64_t last_us= 0;
    int64_t now_us = esp_timer_get_time();
    static float sum_angle = 0.0f;
    float Ts = (now_us - last_us) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    //
    sum_angle = normalizeAngle((sum_angle + target_v*Ts));
    //
    // setPhaseVoltage((-1)*voltage_power_supply/3, 0, electricalAngle(sum_angle));
    setPhaseVoltage(openloop_voltage, 0, electricalAngle(sum_angle));

    last_us = now_us;
    return 0;
}

//
float PID_Controller(float error, PIDController *pid){
    float Kp = pid->Kp;
    float Ki = pid->Ki;
    float Kd = pid->Kd;
    float limit = pid->limit;
    float ramp = pid->ramp;

    // 🔥 核心修复：为电流/位置/速度 分别定义独立静态变量
    static int64_t last_us[3] = {0};
    static float last_error[3] = {0.0f};
    static float error_integral[3] = {0.0f};
    static float last_output[3] = {0.0f};
    
    // 判断当前是哪个PID
    int idx = 0;
    if(pid == &pid[PID_SITE]) idx = 1;       // 位置环
    if(pid == &pid[PID_VELOCITY]) idx = 2;   // 速度环
    if(pid == &pid[PID_CURRENT]) idx = 3;    // 电流环

    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - last_us[idx]) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // 独立积分
    error_integral[idx] += error * Ts;
    if(Ki > 0)
        error_integral[idx] = constrain(error_integral[idx], (-1)*limit/Ki, limit/Ki);
    else
        error_integral[idx] = 0;

    // PID计算
    float output = Kp * error + Ki * error_integral[idx] + Kd *(error - last_error[idx])/Ts;
    output = constrain(output, -limit, limit);

    // 斜坡
    if(ramp > 0){
        if(fabs(output - last_output[idx]) > ramp * Ts) {
            output = last_output[idx] + ((output > last_output[idx]) ? ramp * Ts : -ramp * Ts);
        }
    }

    // 保存独立变量
    last_output[idx] = output;
    last_error[idx] = error; 
    last_us[idx] = now_us;

    return output;
}

// void SiteCloseLoop(float error){
//     float Kp = 0.133; //6/(PI/2)
//     float Ki = 1.0f;    // 
//     float Kd = 0.1f;
//     float limit = 5.0f;
//     float ramp = 10.0f;

//     static int64_t last_us= 0;
//     int64_t now_us = esp_timer_get_time();
//     float Ts = (now_us - last_us) * 1e-6f;
//     if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

//     static float last_error = 0.0f;
//     static float error_integral = 0.0f;
//     static float last_output = 0.0f;
//     error_integral += error;
//     error_integral = constrain(error_integral, (-1)*limit, limit); //
//     float Uq = Kp * Iq_error + Ki * Iq_integral;
//     Uq = constrain(Uq, -6.0f, 6.0f);
//     setPhaseVoltage(Uq, 0, electricalAngle_(current_angle));
// }
