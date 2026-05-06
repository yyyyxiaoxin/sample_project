#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder.h"
#include "inline_current.h"
#include "pwm_init.h"
#include "velocity_openloop.h"

#define TAG "main"

#define PI_UART_NUM          UART_NUM_2
#define PI_UART_BAUD         115200
#define PI_UART_BUF_SIZE     256

#define ALIGN_VOLTAGE        3.0f
#define ALIGN_ANGLE          4.712f
#define CONTROL_PERIOD_MS    10
#define COLLECT_PERIOD_MS    5

#define TRACK_DEADBAND_PX    5.0f
#define TRACK_PIXEL_TO_RAD_X 0.00025f
#define TRACK_PIXEL_TO_RAD_Y 0.00025f
#define TRACK_MAX_STEP_RAD   0.015f
#define TRACK_X_SIGN         1.0f
#define TRACK_Y_SIGN         1.0f
#define GIMBAL_MAX_OFFSET_RAD 1.2f

#define BASE_PID             PID_SITE
#define SIDE_PID             PID_SITE0

curr_sense_t cs_m0;

static QueueHandle_t pi_uart_queue;

static volatile float target_angle[MOTOR_COUNT] = {0.0f, 0.0f};
static float home_angle[MOTOR_COUNT] = {0.0f, 0.0f};
static volatile float current_angle[MOTOR_COUNT] = {0.0f, 0.0f};
static volatile float current_vel[MOTOR_COUNT] = {0.0f, 0.0f};
static volatile float filter_vel[MOTOR_COUNT] = {0.0f, 0.0f};
static volatile float electrical_angle[MOTOR_COUNT] = {0.0f, 0.0f};
static volatile float camera_error_x = 0.0f;
static volatile float camera_error_y = 0.0f;
static volatile int64_t last_camera_us = 0;
static volatile float Iq_actual = 0.0f;

PIDController pid[PID_MAX_COUNT] = {
    [PID_CURRENT] = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0f, .limit = 6.0f, .ramp = 0.0f},
    [PID_SITE] = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, .limit = 4.0f, .ramp = 0.0f},
    [PID_VELOCITY] = {.Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0f, .limit = 6.0f, .ramp = 10000.0f},
    [PID_SITE0] = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, .limit = 4.0f, .ramp = 0.0f},
    [PID_VELOCITY1] = {.Kp = 5.0f, .Ki = 1.0f, .Kd = 0.0f, .limit = 6.0f, .ramp = 10000.0f},
};

static float electrical_angle_for_motor(uint8_t motor, float shaft_angle)
{
    float zero = (motor == MOTOR_BASE) ? zero_electric_angle : zero_electric_angle_m1;
    return normalizeAngle((float)POLE_PAIRS * shaft_angle - zero);
}

static void align_motor(uint8_t motor, as5600_encoder_t *encoder)
{
    ESP_LOGI(TAG, "align motor %u", motor);
    setPhaseVoltageMotor(motor, ALIGN_VOLTAGE, 0.0f, ALIGN_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(3000));

    float zero = normalizeAngle((float)POLE_PAIRS * AS5600_get_angle(encoder));
    if (motor == MOTOR_BASE) {
        zero_electric_angle = zero;
    } else {
        zero_electric_angle_m1 = zero;
    }

    setPhaseVoltageMotor(motor, 0.0f, 0.0f, 0.0f);
    ESP_LOGI(TAG, "motor %u aligned, zero electric angle %.3f", motor, zero);
}

static float cal_Iq(float current_a, float current_b, float angle_el)
{
    float I_alpha = current_a;
    float I_beta = 0.57735f * current_a + 1.1547f * current_b;
    return I_beta * cosf(angle_el) - I_alpha * sinf(angle_el);
}

static void apply_camera_error(float dx, float dy)
{
    camera_error_x = dx;
    camera_error_y = dy;
    last_camera_us = esp_timer_get_time();

    if (fabsf(dx) > TRACK_DEADBAND_PX) {
        float step_x = constrain(TRACK_X_SIGN * dx * TRACK_PIXEL_TO_RAD_X, -TRACK_MAX_STEP_RAD, TRACK_MAX_STEP_RAD);
        target_angle[MOTOR_BASE] += step_x;
    }

    if (fabsf(dy) > TRACK_DEADBAND_PX) {
        float step_y = constrain(TRACK_Y_SIGN * dy * TRACK_PIXEL_TO_RAD_Y, -TRACK_MAX_STEP_RAD, TRACK_MAX_STEP_RAD);
        target_angle[MOTOR_SIDE] += step_y;
    }

    target_angle[MOTOR_BASE] = constrain(target_angle[MOTOR_BASE],
        home_angle[MOTOR_BASE] - GIMBAL_MAX_OFFSET_RAD,
        home_angle[MOTOR_BASE] + GIMBAL_MAX_OFFSET_RAD);
    target_angle[MOTOR_SIDE] = constrain(target_angle[MOTOR_SIDE],
        home_angle[MOTOR_SIDE] - GIMBAL_MAX_OFFSET_RAD,
        home_angle[MOTOR_SIDE] + GIMBAL_MAX_OFFSET_RAD);
}

static bool parse_camera_line(const char *line, float *dx, float *dy)
{
    return sscanf(line, "dx:%f,dy:%f", dx, dy) == 2
        || sscanf(line, "x:%f,y:%f", dx, dy) == 2
        || sscanf(line, "err:%f,%f", dx, dy) == 2
        || sscanf(line, "%f,%f", dx, dy) == 2;
}

static void handle_pi_line(const char *line)
{
    float dx = 0.0f;
    float dy = 0.0f;

    if (parse_camera_line(line, &dx, &dy)) {
        apply_camera_error(dx, dy);
        return;
    }

    if (strncmp(line, "angle0:", strlen("angle0:")) == 0) {
        target_angle[MOTOR_BASE] = strtof(line + strlen("angle0:"), NULL);
    } else if (strncmp(line, "angle1:", strlen("angle1:")) == 0) {
        target_angle[MOTOR_SIDE] = strtof(line + strlen("angle1:"), NULL);
    } else if (strncmp(line, "pid0_p:", strlen("pid0_p:")) == 0) {
        pid[BASE_PID].Kp = strtof(line + strlen("pid0_p:"), NULL);
    } else if (strncmp(line, "pid0_i:", strlen("pid0_i:")) == 0) {
        pid[BASE_PID].Ki = strtof(line + strlen("pid0_i:"), NULL);
    } else if (strncmp(line, "pid0_d:", strlen("pid0_d:")) == 0) {
        pid[BASE_PID].Kd = strtof(line + strlen("pid0_d:"), NULL);
    } else if (strncmp(line, "pid1_p:", strlen("pid1_p:")) == 0) {
        pid[SIDE_PID].Kp = strtof(line + strlen("pid1_p:"), NULL);
    } else if (strncmp(line, "pid1_i:", strlen("pid1_i:")) == 0) {
        pid[SIDE_PID].Ki = strtof(line + strlen("pid1_i:"), NULL);
    } else if (strncmp(line, "pid1_d:", strlen("pid1_d:")) == 0) {
        pid[SIDE_PID].Kd = strtof(line + strlen("pid1_d:"), NULL);
    } else {
        ESP_LOGW("pi_uart", "unknown line: %s", line);
    }
}

static void pi_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = PI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(PI_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PI_UART_NUM, PI_UART_TX_GPIO, PI_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(PI_UART_NUM, PI_UART_BUF_SIZE, PI_UART_BUF_SIZE, 20, &pi_uart_queue, 0));

    ESP_LOGI("pi_uart", "UART2 ready: TX=%d RX=%d baud=%d", PI_UART_TX_GPIO, PI_UART_RX_GPIO, PI_UART_BAUD);
}

static void pi_uart_receive_task(void *arg)
{
    (void)arg;
    uart_event_t event;
    uint8_t data[PI_UART_BUF_SIZE] = {0};
    char line[PI_UART_BUF_SIZE] = {0};
    size_t line_len = 0;

    while (1) {
        if (xQueueReceive(pi_uart_queue, &event, portMAX_DELAY) && event.type == UART_DATA) {
            int to_read = event.size;
            if (to_read > PI_UART_BUF_SIZE - 1) to_read = PI_UART_BUF_SIZE - 1;

            int len = uart_read_bytes(PI_UART_NUM, data, to_read, pdMS_TO_TICKS(10));
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                if (c == '\r' || c == '\n') {
                    if (line_len > 0) {
                        line[line_len] = '\0';
                        handle_pi_line(line);
                        line_len = 0;
                    }
                } else if (line_len < sizeof(line) - 1) {
                    line[line_len++] = c;
                } else {
                    line_len = 0;
                }
            }
        }
    }
}

static void collect_task(void *arg)
{
    (void)arg;

    while (1) {
        curr_sense_get_currents(&cs_m0);

        current_vel[MOTOR_BASE] = AS5600_get_velocity(&encoder_m0);
        current_angle[MOTOR_BASE] = encoder_m0.velocity_last_angle;
        filter_vel[MOTOR_BASE] = LowPassFilterEncoder(&encoder_m0, current_vel[MOTOR_BASE]);
        electrical_angle[MOTOR_BASE] = electrical_angle_for_motor(MOTOR_BASE, current_angle[MOTOR_BASE]);
        Iq_actual = cal_Iq(cs_m0.current_a, cs_m0.current_b, electrical_angle[MOTOR_BASE]);

        current_vel[MOTOR_SIDE] = AS5600_get_velocity(&encoder_m1);
        current_angle[MOTOR_SIDE] = encoder_m1.velocity_last_angle;
        filter_vel[MOTOR_SIDE] = LowPassFilterEncoder(&encoder_m1, current_vel[MOTOR_SIDE]);
        electrical_angle[MOTOR_SIDE] = electrical_angle_for_motor(MOTOR_SIDE, current_angle[MOTOR_SIDE]);

        vTaskDelay(pdMS_TO_TICKS(COLLECT_PERIOD_MS));
    }
}

static void gimbal_control_task(void *arg)
{
    (void)arg;

    while (1) {
        float base_uq = PID_Controller(target_angle[MOTOR_BASE] - current_angle[MOTOR_BASE], &pid[BASE_PID]);
        float side_uq = PID_Controller(target_angle[MOTOR_SIDE] - current_angle[MOTOR_SIDE], &pid[SIDE_PID]);

        setPhaseVoltageMotor(MOTOR_BASE, base_uq, 0.0f, electrical_angle[MOTOR_BASE]);
        setPhaseVoltageMotor(MOTOR_SIDE, side_uq, 0.0f, electrical_angle[MOTOR_SIDE]);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void log_task(void *arg)
{
    (void)arg;

    while (1) {
        ESP_LOGI("log", "cam dx=%.1f dy=%.1f age=%lldms",
            camera_error_x,
            camera_error_y,
            (long long)((esp_timer_get_time() - last_camera_us) / 1000));
        ESP_LOGI("log", "m0 target=%.2f angle=%.2f vel=%.2f",
            target_angle[MOTOR_BASE],
            current_angle[MOTOR_BASE],
            filter_vel[MOTOR_BASE]);
        ESP_LOGI("log", "m1 target=%.2f angle=%.2f vel=%.2f",
            target_angle[MOTOR_SIDE],
            current_angle[MOTOR_SIDE],
            filter_vel[MOTOR_SIDE]);
        ESP_LOGI("log", "m0 current a=%.2f b=%.2f c=%.2f iq=%.2f",
            cs_m0.current_a,
            cs_m0.current_b,
            cs_m0.current_c,
            Iq_actual);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(AS5600_encoder_init(&encoder_m0, I2C_NUM_0, I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_ERROR_CHECK(AS5600_encoder_init(&encoder_m1, I2C_NUM_1, I2C2_SDA_GPIO, I2C2_SCL_GPIO));

    foc_pwm_init();
    curr_sense_init();
    pi_uart_init();

    align_motor(MOTOR_BASE, &encoder_m0);
    align_motor(MOTOR_SIDE, &encoder_m1);

    current_angle[MOTOR_BASE] = AS5600_get_angle_total(&encoder_m0);
    current_angle[MOTOR_SIDE] = AS5600_get_angle_total(&encoder_m1);
    target_angle[MOTOR_BASE] = current_angle[MOTOR_BASE];
    target_angle[MOTOR_SIDE] = current_angle[MOTOR_SIDE];
    home_angle[MOTOR_BASE] = current_angle[MOTOR_BASE];
    home_angle[MOTOR_SIDE] = current_angle[MOTOR_SIDE];
    electrical_angle[MOTOR_BASE] = electrical_angle_for_motor(MOTOR_BASE, current_angle[MOTOR_BASE]);
    electrical_angle[MOTOR_SIDE] = electrical_angle_for_motor(MOTOR_SIDE, current_angle[MOTOR_SIDE]);
    last_camera_us = esp_timer_get_time();

    xTaskCreatePinnedToCore(gimbal_control_task, "gimbal_ctrl", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(collect_task, "collect", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(log_task, "log_task", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(pi_uart_receive_task, "pi_uart_rx", 4096, NULL, 10, NULL, 0);
}
