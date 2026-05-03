#include <stdio.h>
#include <math.h>
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

#define TAG "main"

#define TARGET_VELOCITY    5.0f
#define TARGET_ANGLE       3.14

#define _3PI_2        4.712f  // 灯哥寻零用的固定角度

float electricalAngle_(float shaft_single_angle) {
    // return normalizeAngle((float)((-1) * POLE_PAIRS) * shaft_single_angle - zero_electric_angle);
    return normalizeAngle((float)(1 * POLE_PAIRS) * shaft_single_angle - zero_electric_angle);
}

static float target_velocity = TARGET_VELOCITY;
static float target_angle = TARGET_ANGLE;
static float Iq_target = 0.3f;

volatile static float electricalAngle = 0;
volatile static float current_angle = 0;
volatile static float current_vel = 0;
volatile static float filter_vel = 0;
volatile static float Iq_actual = 0;


#define PID_SET PID_SITE0
// float Kp = 0.133; //6/(PI/2)
// float Ki = 1.0f;    // 
// float Kp = 0.4f;    



float cal_Iq_Id(float current_a, float current_b, float angle_el);

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

void velocity_openloop_task(void *arg)
{
    while(1)
    {
        velocityOpenloop(TARGET_VELOCITY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void collect_task(void *arg)
{
    while(1)
    {
        curr_sense_get_currents(&cs_m0);
        Iq_actual = cal_Iq_Id(cs_m0.current_a, cs_m0.current_b, electricalAngle_(current_angle));
        current_angle = getAngle_total();
        electricalAngle = electricalAngle_(current_angle);
        current_vel =  get_velocity();
        filter_vel = LowPassFilter(current_vel);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void log_task(void *arg)
{
    while(1)
    {
        ESP_LOGI("log","target_vel:%.2f , current_angle: %.2f, current_vel: %.2f,filter_vel: %.2f",
            target_velocity,current_angle,current_vel,filter_vel);
        ESP_LOGI("log","current_a:%.2f , current_b: %.2f, current_c: %.2f,Iq_actual: %.2f",
            cs_m0.current_a,cs_m0.current_b,cs_m0.current_c,Iq_actual);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    ESP_LOGI("UART", "支持指令: velocity:x  angle:x  voltage:x");
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
               if (strstr(cmd_str, "velocity:") != NULL) {
                    // 跳过前缀，提取后面的数值
                    float val = atof(cmd_str + strlen("velocity:"));
                    target_velocity = val;
                    ESP_LOGI("UART", "✅ 目标速度设置: %.2f", target_velocity);
                }
                // 2. 解析 angle:xxx  设置目标角度
                else if (strstr(cmd_str, "angle:") != NULL) {
                    float val = atof(cmd_str + strlen("angle:"));
                    target_angle = val;
                    ESP_LOGI("UART", "✅ 目标角度设置: %.2f", target_angle);
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
    [PID_SITE]     = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0, .limit = 8.0f, .ramp = 0.0f},
    // [PID_SITE]     = {.Kp = 2.0f, .Ki = 0.8f, .Kd = 0.0, .limit = 8.0f, .ramp = 0.0f},
    [PID_VELOCITY] = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0, .limit = 6.0f, .ramp = 10000.0f},
    [PID_SITE0] = {.Kp = 2.0f, .Ki = 0.5f, .Kd = 0.0, .limit = 2.5f, .ramp = 0.0f},
    // [PID_SITE0] = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0, .limit = 8.0f, .ramp = 0.0f},
    [PID_VELOCITY1] = {.Kp = 5.0f, .Ki = 1.0f, .Kd = 0.0, .limit = 6.0f, .ramp = 10000.0f},
    // [PID_VELOCITY1] = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0, .limit = 6.0f, .ramp = 10000.0f},
};

void site_closeloop_task(void *arg)
{
    while(1)
    {
        // setPhaseVoltage(constrain(Uq,-5,5) , 0, electricalAngle(normalizeAngle(getAngle_total())));
        // setPhaseVoltage((-1)*PID_Controller((target_angle - current_angle), &pid[PID_SITE]), 0, electricalAngle);
        setPhaseVoltage(PID_Controller((target_angle - current_angle), &pid[PID_SITE]), 0, electricalAngle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void velocity_closeloop_task(void *arg)
{
    while(1)
    {
        // setPhaseVoltage(constrain(Uq,-5,5) , 0, electricalAngle(normalizeAngle(getAngle_total())));
        setPhaseVoltage((-1)*PID_Controller((target_velocity - filter_vel), &pid[PID_VELOCITY]), 0, electricalAngle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void site_velocity_closeloop_task(void *arg)
{
    while(1)
    {
        setPhaseVoltage(-1*PID_Controller(((-1)*PID_Controller((target_angle - current_angle), &pid[PID_SITE0]) - filter_vel), &pid[PID_VELOCITY1]), 0, electricalAngle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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

void current_loop_task(void *arg)
{
    while(1)
    {
        // setPhaseVoltage(constrain(Uq,-5,5) , 0,
        // setPhaseVoltage(PID_Controller((Iq_target - Iq_actual), &pid[PID_CURRENT]), 0, electricalAngle);    
        setPhaseVoltage(-1*PID_Controller((Iq_target - Iq_actual), &pid[PID_CURRENT]), 0, electricalAngle);    
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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
    AS5600_init(I2C_SDA_GPIO , I2C_SCL_GPIO);
    foc_pwm_init();
    curr_sense_init();
    uart_init();

    ESP_LOGI(TAG, "开始对齐转子...");
    setPhaseVoltage(3.0f, 0, _3PI_2);  // 固定出力对齐
    vTaskDelay(pdMS_TO_TICKS(3000));  // 保持3秒
    zero_electric_angle = electricalAngle_(getAngle()); // 读取零电角度
    setPhaseVoltage(0, 0, 0);         // 停止出力
    ESP_LOGI(TAG, "对齐完成！零电角度: %.2f", zero_electric_angle);

    // xTaskCreate(sit_closeloop_task, "foc_task", 4096, NULL, 5, NULL);
    // xTaskCreatePinnedToCore(velocity_closeloop_task, "velocity_task", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(velocity_openloop_task, "velocity_task", 4096, NULL, 6, NULL,0);
    xTaskCreatePinnedToCore(site_closeloop_task, "current_loop_task", 4096, NULL, 6, NULL,0);
    // xTaskCreatePinnedToCore(site_velocity_closeloop_task, "current_loop_task", 4096, NULL, 6, NULL,0);
    xTaskCreatePinnedToCore(collect_task, "angle_task", 4096, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(log_task, "log_task", 2048, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(uart_receive_task, "uart_rx_task", 2048, NULL, 10, NULL,0);
}





