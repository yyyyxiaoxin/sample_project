#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "encoder.h"
#include "pwm_init.h"
#include "velocity_openloop.h"
#include "inline_current.h"

// 定义电流对象
curr_sense_t cs_m0;
curr_sense_t cs_m1;

#define TAG "main"

#define TARGET_VELOCITY    5.0f
#define TARGET_ANGLE       3.14

#define _3PI_2        4.712f  // 灯哥寻零用的固定角度

static float M0_sensor_dir = -1.0f;
static float M1_sensor_dir = 1.0f;
static float M0_torque_dir = -1.0f;
static float M1_torque_dir = 1.0f;
static float M0_foc_phase_offset = PI * 0.5f;
static float M1_foc_phase_offset = PI * 0.5f;

static float motor_sensor_dir(char motor)
{
    return (motor == 0) ? M0_sensor_dir : M1_sensor_dir;
}

float electricalAngle_(char motor, float shaft_single_angle) {
    // return normalizeAngle((float)((-1) * POLE_PAIRS) * shaft_single_angle - zero_electric_angle);
    return normalizeAngle(motor_sensor_dir(motor) * POLE_PAIRS * shaft_single_angle - ((motor == 0) ? M0_zero_electric_angle : M1_zero_electric_angle));
}

static float angle_diff(float a, float b)
{
    float diff = normalizeAngle(a - b);
    if (diff > PI) {
        diff -= 2.0f * PI;
    }
    return diff;
}

static float target_velocity = TARGET_VELOCITY;
static float target_angle = TARGET_ANGLE;
static float M0_target_speed = 0.8f;
static float M1_target_speed = 3.0f;
static float M0_target_angle = 0.0f;
static float M1_target_angle = 0.0f;
static float M1_target_vel_ff = 0.0f;
static bool M1_sweep_enable = true;
static float M1_sweep_speed = 0.2f;
static float M1_sweep_amplitude = 5.0f * PI / 180.0f;
static float M1_sweep_center = 2.0f;
static float M1_sweep_center_offset = 0.0f;
static float M1_sweep_phase = 0.0f;
static int64_t M1_sweep_last_us = 0;
static float M0_position_limit = PI * 3.0f;
static float M1_position_limit = PI * 3.0f;
static float Iq_target = 0.3f;

volatile static float M0_electricalAngle = 0;
volatile static float M1_electricalAngle = 0;
volatile static float M0_current_angle = 0;
volatile static float M1_current_angle = 0;
volatile static float M0_current_vel = 0;
volatile static float M1_current_vel = 0;
volatile static float M0_filter_vel = 0;
volatile static float M1_filter_vel = 0;
volatile static float M0_Iq_actual = 0;
volatile static float M1_Iq_actual = 0;
volatile static uint16_t M0_encoder_fail_count = 0;
volatile static uint16_t M1_encoder_fail_count = 0;


#define PID_SET PID_SITE0
// float Kp = 0.133; //6/(PI/2)
// float Ki = 1.0f;    // 
// float Kp = 0.4f;    



float cal_Iq_Id(float current_a, float current_b, float angle_el);

static void calibrate_m1_electrical_angle(void)
{
    float start_angle = getAngle_total(1);
    float openloop_angle = 0.0f;

    ESP_LOGI(TAG, "M1 calibrating sensor direction...");
    for (int i = 0; i < 180; i++) {
        setPhaseVoltage(1, 2.8f, 0, openloop_angle);
        openloop_angle = normalizeAngle(openloop_angle + 0.018f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float end_angle = getAngle_total(1);
    float delta_angle = end_angle - start_angle;
    M1_sensor_dir = (delta_angle >= 0.0f) ? 1.0f : -1.0f;
    ESP_LOGI(TAG, "M1 sensor dir: %.0f, calib delta: %.2f", M1_sensor_dir, delta_angle);

    ESP_LOGI(TAG, "M1 calibrating zero electrical angle...");
    setPhaseVoltage(1, 3.2f, 0, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(2500));
    M1_zero_electric_angle = normalizeAngle(M1_sensor_dir * POLE_PAIRS * getAngle(1) - _3PI_2);
    setPhaseVoltage(1, 0, 0, 0);
    ESP_LOGI(TAG, "M1 zero electrical angle: %.2f", M1_zero_electric_angle);
}

static float velocity_from_angle(char motor, float angle)
{
    int idx = (motor == 0) ? 0 : 1;
    static int64_t last_us[2] = {0, 0};
    static float last_angle[2] = {0.0f, 0.0f};

    int64_t now_us = esp_timer_get_time();
    if (last_us[idx] == 0) {
        last_angle[idx] = angle;
        last_us[idx] = now_us;
        return 0.0f;
    }

    float Ts = (now_us - last_us[idx]) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) {
        Ts = 5e-3f;
    }

    float vel = (angle - last_angle[idx]) / Ts;
    last_angle[idx] = angle;
    last_us[idx] = now_us;
    return vel;
}

// void sit_closeloop_task(void *arg)
// {
//     static float Iq_integral = 0.0f; 
//     while(1)
//     {
//         float Iq_target = 0.3f;

//         float Iq_error = Iq_target - Iq_actual;

//         // 4. 积分计算（抗饱和：积分限幅，防止失控）
//         Iq_integral += Iq_error * 0.01f;
//         Iq_integral = constrain(Iq_integral, -5.0f, 5.0f);

//         // 5. 纯手写PI计算 → 直接算出Uq
//         float Uq = 1.2f * Iq_error + 0.5f * Iq_integral;
//         // 6. Uq限幅（保护电机）
//         Uq = constrain(Uq, -6.0f, 6.0f);

//         // 7. 直接驱动电机（删掉错误的*180/PI！）
//         setPhaseVoltage(Uq, 0, electricalAngle_(current_angle));
        
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }


// #define PID_TS  0.01f  
// static float Iq_integral = 0.0f;    // 电流环积分
// static float pos_integral = 0.0f;   // 位置环积分
// float pos_Kp = 0.3f;
// float pos_Ki = 0.2f;
// // 2. 电流环参数（控力矩/力气）
// float curr_Kp = 1.0f;
// float curr_Ki = 0.5f;

// void sit_closeloop_task(void *arg)
// {
//     while(1)
//     {
//         // ================ 第1层：位置环 → 计算目标力矩（力位核心！） ================
//         float angle_error = TARGET_ANGLE - normalizeAngle(current_angle);
//         // 位置环积分（抗饱和）
//         pos_integral += angle_error * PID_TS;
//         pos_integral = constrain(pos_integral, -1.0f, 1.0f);
//         // 位置PID输出 = 目标力矩电流 Iq_target
//         float Iq_target = pos_Kp * angle_error + pos_Ki * pos_integral;
//         // 限制目标力矩（防止出力太大）
//         Iq_target = constrain(Iq_target, -0.2f, 0.2f); 

//         // ================ 第2层：电流环 → 跟踪目标力矩 ================
//         float Iq_actual = cal_Iq_Id(cs_m0.current_a, cs_m0.current_b, electricalAngle_(current_angle));
//         float Iq_error = Iq_target - Iq_actual;
//         // 电流环积分
//         Iq_integral += Iq_error * PID_TS;
//         Iq_integral = constrain(Iq_integral, -5.0f, 5.0f);
//         // 电流PI计算 → Uq
//         float Uq = curr_Kp * Iq_error + curr_Ki * Iq_integral;
//         Uq = constrain(Uq, -6.0f, 6.0f);

//         // ================ 驱动电机 ================
//         setPhaseVoltage(Uq, 0, electricalAngle_(current_angle));
        
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void velocity_closeloop_task(void *arg)
// {
//     static float vel_integral = 0.0f;
//     static float vel_error = 0.0f;
//     while(1)
//     {
//         vel_error= TARGET_VELOCITY - filter_vel;  
//         vel_integral += vel_error * 1e-3f * 10; 
//         // vel_integral =constrain(vel_integral, -10.0f, 10.0f);       
//         setPhaseVoltage(constrain(Kp * vel_error + Ki * vel_integral, -6, 6), 0.0f, electricalAngle_(current_angle));       
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }


// void velocity_openloop_task(void *arg)
// {
//     while(1)
//     {
//         velocityOpenloop(TARGET_VELOCITY);
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
void collect_task(void *arg)
{
    while(1)
    {
        float M0_angle_new = 0.0f;
        bool M0_angle_ok = (getAngle_total_checked(0, &M0_angle_new) == ESP_OK);
        if (M0_angle_ok) {
            M0_encoder_fail_count = 0;
            M0_current_angle = M0_angle_new;
            M0_electricalAngle = electricalAngle_(0, M0_current_angle);
        } else if (M0_encoder_fail_count < 1000) {
            M0_encoder_fail_count++;
        }

        float M1_angle_new = 0.0f;
        bool M1_angle_ok = (getAngle_total_checked(1, &M1_angle_new) == ESP_OK);
        if (M1_angle_ok) {
            M1_encoder_fail_count = 0;
            M1_current_angle = M1_angle_new;
            M1_electricalAngle = electricalAngle_(1, M1_current_angle);
        } else if (M1_encoder_fail_count < 1000) {
            M1_encoder_fail_count++;
        }

        curr_sense_get_currents(0, &cs_m0);
        curr_sense_get_currents(1, &cs_m1);
        M0_Iq_actual = cal_Iq_Id(cs_m0.current_a, cs_m0.current_b, M0_electricalAngle);
        M1_Iq_actual = cal_Iq_Id(cs_m1.current_a, cs_m1.current_b, M1_electricalAngle);

        if (M0_angle_ok) {
            M0_current_vel = velocity_from_angle(0, M0_current_angle);
            M0_filter_vel = LowPassFilter(0, M0_current_vel);
        }
        if (M1_angle_ok) {
            M1_current_vel = velocity_from_angle(1, M1_current_angle);
            M1_filter_vel = LowPassFilter(1, M1_current_vel);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void log_task(void *arg)
{
    while(1)
    {
        ESP_LOGI("M0","target_vel: %.2f, angle: %.2f, vel: %.2f",
            M0_target_speed,M0_current_angle,M0_filter_vel);
        ESP_LOGI("M1","target: %.2f, angle: %.2f, vel: %.2f",
            M1_target_angle,M1_current_angle,M1_filter_vel);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// 共用变量（你原代码里的全局变量，保留）
extern float openloop_voltage;
extern float voltage_power_supply;

static QueueHandle_t uart_queue;

// UART初始化（使用UART0，电脑USB默认串口，无自定义引脚）
void uart_init() {

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // 1. 配置UART0（电脑USB串口）
    uart_param_config(UART_NUM_0, &uart_config);
    // 2. 使用UART0默认硬件引脚(GPIO1=TX/GPIO3=RX)，无需修改
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);   
    // 3. 安装UART0驱动
    uart_driver_install(UART_NUM_0, 1024 , 1024, 20, &uart_queue, 0);

    openloop_voltage = voltage_power_supply / 4.0f; 
    ESP_LOGI("UART", "UART初始化完成");
    ESP_LOGI("UART", "支持指令: velocity:x  m0_velocity:x  angle:x  m0_angle:x  m1_angle:x  m1_sweep:x  m1_sweep_speed:x  m1_center_offset:x  voltage:x");
    ESP_LOGI("UART", "PID调参:pid_p:x  pid_i:x  pid_d:x  pid_limit:x  pid_ramp:x");
}

PIDController pid[PID_MAX_COUNT];
// UART接收任务（监听电脑USB发送的指令）
void uart_receive_task(void *arg) {
    uart_event_t uart_event;
    uint8_t data[32] = {0};
    char *cmd_str = (char *)data;

    while (1) {
        if (xQueueReceive(uart_queue, &uart_event, portMAX_DELAY)) {
            // 监听UART0数据
            if (uart_event.type == UART_DATA) {
                int len = uart_read_bytes(UART_NUM_0, data, uart_event.size, pdMS_TO_TICKS(10));
                if (len <= 0) {
                    memset(data, 0, sizeof(data));
                    continue;
                }
                for (int i = 0; i < len; i++) {
                    if (data[i] == '\r' || data[i] == '\n') data[i] = '\0';
                }

                // 字符串转浮点数
               if (strstr(cmd_str, "m0_velocity:") != NULL) {
                    float val = atof(cmd_str + strlen("m0_velocity:"));
                    M0_target_speed = constrain(val, -8.0f, 8.0f);
                    ESP_LOGI("UART", "M0 target velocity: %.2f", M0_target_speed);
                }
                else if (strstr(cmd_str, "m1_velocity:") != NULL) {
                    float val = atof(cmd_str + strlen("m1_velocity:"));
                    M1_target_speed = constrain(val, 0.0f, 6.28f);
                    ESP_LOGI("UART", "M1 target velocity: %.2f", M1_target_speed);
                }
                else if (strstr(cmd_str, "velocity:") != NULL) {
                    // 跳过前缀，提取后面的数值
                    float val = atof(cmd_str + strlen("velocity:"));
                    target_velocity = val;
                    ESP_LOGI("UART", "✅ 目标速度设置: %.2f", target_velocity);
                }
                // 2. 解析 angle:xxx  设置目标角度
                else if (strstr(cmd_str, "m0_angle:") != NULL) {
                    float val = atof(cmd_str + strlen("m0_angle:"));
                    M0_target_angle = val;
                    ESP_LOGI("UART", "M0 target angle: %.2f", M0_target_angle);
                }
                else if (strstr(cmd_str, "m1_angle:") != NULL) {
                    float val = atof(cmd_str + strlen("m1_angle:"));
                    M1_sweep_enable = false;
                    M1_target_vel_ff = 0.0f;
                    M1_target_angle = val;
                    ESP_LOGI("UART", "M1 target angle: %.2f", M1_target_angle);
                }
                else if (strstr(cmd_str, "angle:") != NULL) {
                    float val = atof(cmd_str + strlen("angle:"));
                    target_angle = val;
                    M0_target_angle = val;
                    M1_sweep_enable = false;
                    M1_target_vel_ff = 0.0f;
                    M1_target_angle = val;
                    ESP_LOGI("UART", "✅ 目标角度设置: %.2f", target_angle);
                }
                else if (strstr(cmd_str, "m1_sweep:") != NULL) {
                    float val = atof(cmd_str + strlen("m1_sweep:"));
                    M1_sweep_enable = (val > 0.5f);
                    M1_target_vel_ff = 0.0f;
                    if (M1_sweep_enable) {
                        M1_sweep_center = 2.0f + M1_sweep_center_offset;
                        M1_sweep_phase = 0.0f;
                        M1_sweep_last_us = 0;
                    } else {
                        M1_target_angle = M1_current_angle;
                    }
                    ESP_LOGI("UART", "M1 sweep: %d", M1_sweep_enable ? 1 : 0);
                }
                else if (strstr(cmd_str, "m1_sweep_speed:") != NULL) {
                    float val = atof(cmd_str + strlen("m1_sweep_speed:"));
                    M1_sweep_speed = constrain(val, 0.2f, 8.0f);
                    ESP_LOGI("UART", "M1 sweep max velocity: %.2f", M1_sweep_speed);
                }
                else if (strstr(cmd_str, "m1_center_offset:") != NULL) {
                    float val = atof(cmd_str + strlen("m1_center_offset:"));
                    M1_sweep_center_offset = constrain(val, -PI, PI);
                    M1_sweep_center = 2.0f + M1_sweep_center_offset;
                    M1_sweep_phase = 0.0f;
                    M1_sweep_last_us = 0;
                    ESP_LOGI("UART", "M1 sweep center offset: %.2f", M1_sweep_center_offset);
                }
                // 3. 解析 voltage:xxx  设置开环电压（带安全校验）
                else if (strstr(cmd_str, "voltage:") != NULL) {
                    float val = atof(cmd_str + strlen("voltage:"));
                    if (val >= 0.0f && val <= voltage_power_supply) {
                        openloop_voltage = val;
                        ESP_LOGI("UART", "✅ 开环电压设置: %.2f V", openloop_voltage);
                    } else {
                        ESP_LOGW("UART", "❌ 电压无效: %.2f V(范围0~%.2f)", val, voltage_power_supply);
                    }
                }
                else if (strstr(cmd_str, "Iq:") != NULL) {
                    float val = atof(cmd_str + strlen("Iq:"));
                    Iq_target = val;
                    ESP_LOGI("UART", "✅ 目标Iq: %.2f A", Iq_target);
                }
                // 设置Kp
                else if (strstr(cmd_str, "pid_p:") != NULL) {
                    float val = atof(cmd_str + strlen("pid_p:"));
                    pid[PID_SET].Kp = val;
                    ESP_LOGI("UART", "✅ 速度环Kp: %.3f", pid[PID_SET].Kp);
                }
                // 设置Ki
                else if (strstr(cmd_str, "pid_i:") != NULL) {
                    float val = atof(cmd_str + strlen("pid_i:"));
                    pid[PID_SET].Ki = val;
                    // 重置积分，防止参数修改后积分饱和
                    // pid[PID_SET].error_integral = 0;
                    ESP_LOGI("UART", "✅ 速度环Ki: %.3f", pid[PID_SET].Ki);
                }
                // 设置Kd
                else if (strstr(cmd_str, "pid_d:") != NULL) {
                    float val = atof(cmd_str + strlen("pid_d:"));
                    pid[PID_SET].Kd = val;
                    ESP_LOGI("UART", "✅ 速度环Kd: %.3f", pid[PID_SET].Kd);
                }
                // 设置输出限幅
                else if (strstr(cmd_str, "pid_limit:") != NULL) {
                    float val = atof(cmd_str + strlen("pid_limit:"));
                    pid[PID_SET].limit = val;
                    ESP_LOGI("UART", "✅ 速度环限幅: %.2f", pid[PID_SET].limit);
                }
                // 设置输出斜坡
                else if (strstr(cmd_str, "pid_ramp:") != NULL) {
                    float val = atof(cmd_str + strlen("pid_ramp:"));
                    pid[PID_SET].ramp = val;
                    ESP_LOGI("UART", "✅ 速度环斜坡: %.2f", pid[PID_SET].ramp);
                }

                // 无效指令
                else {
                    ESP_LOGW("UART", "❌ 未知指令: %s", cmd_str);
                }
                // ==========================================================

                // 清空缓冲区
                memset(data, 0, sizeof(data));
            }
        }
    }
}

// FOC 电流计算（直接用 cs_m0.current_a / current_b）
float cal_Iq_Id(float current_a, float current_b, float angle_el)
{
    float I_alpha = current_a;
    float I_beta  = 0.57735f * current_a + 1.1547f * current_b;

    float ct = cos(angle_el);
    float st = sin(angle_el);
    float I_q = I_beta * ct - I_alpha * st;
    return I_q;
}


PIDController pid[PID_MAX_COUNT] = {
    [PID_CURRENT]  = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0, .limit = 6.0f, .ramp = 0.0f},
    [PID_SITE]     = {.Kp = 3.0f, .Ki = 0.0f, .Kd = 0.0, .limit = 5.5f, .ramp = 0.0f},
    // [PID_SITE]     = {.Kp = 2.0f, .Ki = 0.8f, .Kd = 0.0, .limit = 8.0f, .ramp = 0.0f},
    [PID_VELOCITY] = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0, .limit = 1.2f, .ramp = 20.0f},
    [PID_SITE0] = {.Kp = 2.0f, .Ki = 0.5f, .Kd = 0.0, .limit = 2.5f, .ramp = 0.0f},
    // [PID_SITE0] = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0, .limit = 8.0f, .ramp = 0.0f},
    [PID_VELOCITY1] = {.Kp = 5.0f, .Ki = 1.0f, .Kd = 0.0, .limit = 6.0f, .ramp = 10000.0f},
    // [PID_VELOCITY1] = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0, .limit = 6.0f, .ramp = 10000.0f},
    [PID_SITE1] = {.Kp = 1.2f, .Ki = 0.0f, .Kd = 0.0f, .limit = 2.5f, .ramp = 0.0f},
};

void site_closeloop_task(void *arg)
{
    static float M0_Uq = 0.0f;
    static float M0_vel_integral = 0.0f;
    static float M0_last_target_speed = 0.0f;
    static float M1_Uq = 0.0f;
    int settle_count = 0;
    while(1)
    {
        if (settle_count < 50) {
            M0_Uq = 0.0f;
            M0_vel_integral = 0.0f;
            M0_last_target_speed = M0_target_speed;
            M1_Uq = 0.0f;
            M0_filter_vel = 0.0f;
            M1_filter_vel = 0.0f;
            M0_current_vel = 0.0f;
            M1_current_vel = 0.0f;
            M0_target_angle = M0_current_angle;
            M1_target_angle = M1_current_angle;
            M1_sweep_center = 2.0f + M1_sweep_center_offset;
            M1_target_vel_ff = 0.0f;
            M1_sweep_phase = 0.0f;
            M1_sweep_last_us = 0;
            setPhaseVoltage(0, 0.0f, 0, normalizeAngle(M0_electricalAngle + M0_foc_phase_offset));
            setPhaseVoltage(1, 0.0f, 0, normalizeAngle(M1_electricalAngle + M1_foc_phase_offset));
            settle_count++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        float M0_speed = M0_filter_vel;
        if (fabsf(M0_target_speed - M0_last_target_speed) > 0.05f) {
            M0_vel_integral = 0.0f;
            M0_Uq = 0.0f;
            M0_last_target_speed = M0_target_speed;
        }

        float M0_vel_error = M0_target_speed - M0_speed;
        M0_vel_integral += M0_vel_error * 0.01f;
        M0_vel_integral = constrain(M0_vel_integral, -2.2f, 2.2f);
        float M0_vel_ff = 0.0f;
        if (fabsf(M0_target_speed) > 0.2f) {
            M0_vel_ff = (M0_target_speed > 0.0f) ? 0.80f : -0.80f;
        }
        float M0_Uq_target = M0_vel_ff + 0.42f * M0_vel_error + 0.18f * M0_vel_integral;
        if (fabsf(M0_target_speed) > 0.2f && fabsf(M0_speed) < fabsf(M0_target_speed) * 0.8f) {
            float M0_boost = constrain(0.40f * fabsf(M0_target_speed), 0.60f, 1.25f);
            M0_Uq_target += (M0_target_speed > 0.0f) ? M0_boost : -M0_boost;
        }

        if (M0_encoder_fail_count > 20) {
            M0_Uq_target = 0.0f;
            M0_Uq = 0.0f;
            M0_vel_integral = 0.0f;
        }

        M0_Uq_target = constrain(M0_Uq_target, -5.0f, 5.0f);
        M0_Uq += constrain(M0_Uq_target - M0_Uq, -0.10f, 0.10f);
        setPhaseVoltage(0, M0_torque_dir * M0_Uq, 0, normalizeAngle(M0_electricalAngle + M0_foc_phase_offset));

        if (M1_sweep_enable) {
            int64_t now_us = esp_timer_get_time();
            if (M1_sweep_last_us == 0) {
                M1_sweep_center = 2.0f + M1_sweep_center_offset;
                M1_sweep_phase = 0.0f;
                M1_target_angle = M1_sweep_center;
                M1_target_vel_ff = 0.0f;
                M1_sweep_last_us = now_us;
            } else {
                float Ts = (now_us - M1_sweep_last_us) * 1e-6f;
                if (Ts <= 0.0f || Ts > 0.05f) {
                    Ts = 0.01f;
                }
                M1_sweep_last_us = now_us;

                float sweep_omega = M1_sweep_speed / M1_sweep_amplitude;
                M1_sweep_phase += sweep_omega * Ts;
                if (M1_sweep_phase > 2.0f * PI) {
                    M1_sweep_phase -= 2.0f * PI;
                }
                M1_target_angle = M1_sweep_center + M1_sweep_amplitude * sinf(M1_sweep_phase);
                M1_target_vel_ff = M1_sweep_speed * cosf(M1_sweep_phase);
            }
        } else {
            M1_target_vel_ff = 0.0f;
            M1_sweep_phase = 0.0f;
            M1_sweep_last_us = 0;
        }

        float M1_pos_error = constrain(M1_target_angle - M1_current_angle, -PI, PI);
        float M1_speed = M1_sensor_dir * M1_filter_vel;
        float M1_Uq_target = 0.0f;
        if (M1_sweep_enable) {
            M1_Uq_target = 0.45f * M1_target_vel_ff + 0.70f * M1_pos_error - 0.08f * M1_speed;
            if (fabsf(M1_pos_error) > 0.025f || fabsf(M1_target_vel_ff) > 0.05f) {
                M1_Uq_target += (M1_target_vel_ff >= 0.0f) ? 0.45f : -0.45f;
            }
        } else {
            float M1_vel_target = constrain(1.5f * M1_pos_error, -M1_position_limit, M1_position_limit);
            float M1_vel_error = M1_vel_target - M1_speed;
            M1_Uq_target = 0.70f * M1_vel_error + 0.25f * M1_pos_error;
            if (fabsf(M1_pos_error) > 0.08f) {
                M1_Uq_target += (M1_pos_error > 0.0f) ? 0.90f : -0.90f;
            }
        }

        if (fabsf(M1_speed) > 8.0f) {
            M1_Uq_target = constrain(-0.18f * M1_speed, -4.0f, 4.0f);
        }

        if (!M1_sweep_enable && fabsf(M1_pos_error) < 0.08f && fabsf(M1_speed) < 1.0f) {
            M1_Uq_target = 0.0f;
            M1_Uq = 0.0f;
        }

        if (M1_sweep_enable) {
            M1_Uq_target = constrain(M1_Uq_target, -1.8f, 1.8f);
            M1_Uq += constrain(M1_Uq_target - M1_Uq, -0.035f, 0.035f);
        } else {
            M1_Uq_target = constrain(M1_Uq_target, -4.0f, 4.0f);
            M1_Uq += constrain(M1_Uq_target - M1_Uq, -0.07f, 0.07f);
        }
        setPhaseVoltage(1, M1_torque_dir * M1_Uq, 0, normalizeAngle(M1_electricalAngle + M1_foc_phase_offset));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void velocity_closeloop_task(void *arg)
{
    static float M1_vel_integral = 0.0f;
    static float M1_Uq = 0.0f;
    while(1)
    {
        if (M1_encoder_fail_count > 20) {
            M1_Uq = 0.0f;
            M1_vel_integral = 0.0f;
            setPhaseVoltage(1, 0.0f, 0, normalizeAngle(M1_electricalAngle + M1_foc_phase_offset));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        float M1_speed = M1_sensor_dir * M1_filter_vel;
        float M1_vel_error = M1_target_speed - M1_speed;

        M1_vel_integral += M1_vel_error * 0.01f;
        M1_vel_integral = constrain(M1_vel_integral, 0.0f, 2.0f);

        float M1_Uq_target = 1.15f + 0.12f * M1_vel_error + 0.25f * M1_vel_integral;
        if (M1_speed < 0.5f && M1_vel_error > 0.3f) {
            M1_Uq_target += 1.2f;
        }
        if (M1_vel_error < -0.3f) {
            M1_Uq_target = 0.75f;
        }
        M1_Uq_target = constrain(M1_Uq_target, 0.75f, 3.4f);
        M1_Uq += constrain(M1_Uq_target - M1_Uq, -0.03f, 0.08f);

        setPhaseVoltage(1, M1_torque_dir * M1_Uq, 0, normalizeAngle(M1_electricalAngle + M1_foc_phase_offset));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void m1_angle_debug_task(void *arg)
{
    float openloop_el = 0.0f;
    int log_count = 0;

    while(1)
    {
        setPhaseVoltage(1, 2.8f, 0, openloop_el);
        openloop_el = normalizeAngle(openloop_el + 0.030f);

        if (++log_count >= 20) {
            float measured_el = electricalAngle_(1, M1_current_angle);
            float diff = angle_diff(measured_el, openloop_el);
            float raw_angle = getAngle(1);
            ESP_LOGI("M1_DBG",
                "raw: %.2f,total: %.2f,open_el: %.2f,meas_el: %.2f,diff: %.2f,vel: %.2f,dir: %.0f,zero: %.2f",
                raw_angle,
                M1_current_angle,
                openloop_el,
                measured_el,
                diff,
                M1_filter_vel,
                M1_sensor_dir,
                M1_zero_electric_angle);
            log_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void m1_phase_test_task(void *arg)
{
    float angle = 0.0f;
    while(1)
    {
        setPhaseVoltage(1, 1.5f, 0, angle);
        angle = normalizeAngle(angle + 0.01f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void m1_torque_test_task(void *arg)
{
    int count = 0;
    float Uq = 4.0f;
    while(1)
    {
        if (++count >= 300) {
            Uq = -Uq;
            count = 0;
            ESP_LOGI("M1_TORQUE", "switch Uq: %.2f", Uq);
        }
        setPhaseVoltage(1, Uq, 0, normalizeAngle(M1_electricalAngle + M1_foc_phase_offset));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// void site_velocity_closeloop_task(void *arg)
// {
//     while(1)
//     {
//         setPhaseVoltage(-1*PID_Controller(((-1)*PID_Controller((target_angle - current_angle), &pid[PID_SITE0]) - filter_vel), &pid[PID_VELOCITY1]), 0, electricalAngle);
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void site_current_task(void *arg)
// {
//     while(1)
//     {
//         // setPhaseVoltage(constrain(Uq,-5,5) , 0, electricalAngle(normalizeAngle(getAngle_total())));
//         setPhaseVoltage(-1*PID_Controller(PID_Controller((target_angle - current_angle), &pid[PID_SITE]),&pid[PID_CURRENT]), 0, electricalAngle_(current_angle));
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void velocity_current_task(void *arg)
// {
//     while(1)
//     {
//         // setPhaseVoltage(constrain(Uq,-5,5) , 0, electricalAngle(normalizeAngle(getAngle_total())));
//         setPhaseVoltage(PID_Controller(PID_Controller((target_velocity - filter_vel), &pid[PID_VELOCITY]),&pid[PID_CURRENT]), 0, electricalAngle_(current_angle));
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

// void current_loop_task(void *arg)
// {
//     while(1)
//     {
//         // setPhaseVoltage(constrain(Uq,-5,5) , 0,
//         // setPhaseVoltage(PID_Controller((Iq_target - Iq_actual), &pid[PID_CURRENT]), 0, electricalAngle);    
//         setPhaseVoltage(-1*PID_Controller((Iq_target - Iq_actual), &pid[PID_CURRENT]), 0, electricalAngle);    
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void site_velocity_current_task(void *arg)
// {
//     while(1)
//     {
//         // setPhaseVoltage(constrain(Uq,-5,5) , 0, electricalAngle(normalizeAngle(getAngle_total())));
//         setPhaseVoltage(PID_Controller(PID_Controller(PID_Controller((target_angle - current_angle), &pid[PID_SITE]), &pid[PID_VELOCITY]),&pid[PID_CURRENT]), 0, electricalAngle_(current_angle));
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

void app_main()
{
    AS5600_init();
    foc_pwm_init();
    curr_sense_init();
    uart_init();

    ESP_LOGI(TAG, "开始对齐转子...");
    setPhaseVoltage(0, 3.0f, 0, _3PI_2);  // 固定出力对齐
    vTaskDelay(pdMS_TO_TICKS(2000));  // 保持2秒
    M0_zero_electric_angle = normalizeAngle(motor_sensor_dir(0) * POLE_PAIRS * getAngle(0) - _3PI_2); // 读取零电角度
    setPhaseVoltage(0, 0, 0, 0);         // 零力矩中点输出
    ESP_LOGI(TAG, "对齐完成！M0零电角度: %.2f", M0_zero_electric_angle);
    setPhaseVoltage(1, 3.0f, 0, _3PI_2);  // 固定出力对齐
    vTaskDelay(pdMS_TO_TICKS(2000));  // 保持2秒
    M1_zero_electric_angle = normalizeAngle(motor_sensor_dir(1) * POLE_PAIRS * getAngle(1) - _3PI_2); // 读取零电角度
    setPhaseVoltage(1, 0, 0, 0);         // 零力矩中点输出
    ESP_LOGI(TAG, "对齐完成！M1零电角度: %.2f", M1_zero_electric_angle);
    ESP_LOGI(TAG, "M1 zero electrical angle: %.2f", M1_zero_electric_angle);
    float M0_start_angle = 0.0f;
    if (getAngle_total_checked(0, &M0_start_angle) == ESP_OK) {
        M0_current_angle = M0_start_angle;
    } else {
        M0_current_angle = getAngle_total(0);
    }
    M0_electricalAngle = electricalAngle_(0, M0_current_angle);
    M0_target_angle = M0_current_angle;
    ESP_LOGI(TAG, "M0 position hold target: %.2f", M0_target_angle);

    float M1_start_angle = 0.0f;
    if (getAngle_total_checked(1, &M1_start_angle) == ESP_OK) {
        M1_current_angle = M1_start_angle;
    } else {
        M1_current_angle = getAngle_total(1);
    }
    M1_electricalAngle = electricalAngle_(1, M1_current_angle);
    M1_target_angle = M1_current_angle;
    M1_sweep_center = 2.0f + M1_sweep_center_offset;
    ESP_LOGI(TAG, "M1 position hold target: %.2f", M1_target_angle);

    // xTaskCreate(sit_closeloop_task, "foc_task", 4096, NULL, 5, NULL);
    // xTaskCreatePinnedToCore(velocity_closeloop_task, "velocity_task", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(velocity_openloop_task, "velocity_task", 4096, NULL, 6, NULL,0);
    xTaskCreatePinnedToCore(site_closeloop_task, "position_task", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(m1_angle_debug_task, "m1_angle_debug", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(m1_phase_test_task, "m1_phase_test", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(m1_torque_test_task, "m1_torque_test", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(site_velocity_closeloop_task, "current_loop_task", 4096, NULL, 6, NULL,0);
    xTaskCreatePinnedToCore(collect_task, "angle_task", 4096, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(log_task, "log_task", 2048, NULL, 1, NULL,0);
    xTaskCreatePinnedToCore(uart_receive_task, "uart_rx_task", 2048, NULL, 10, NULL,0);
}





