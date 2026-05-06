#include "pwm_init.h"

#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

float voltage_power_supply = 11.1f;
float Ualpha = 0, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;
float zero_electric_angle = 0.0f;
float zero_electric_angle_m1 = 0.0f;

#define TAG "init"

static const gpio_num_t motor_pwm_gpio[MOTOR_COUNT][3] = {
    [MOTOR_BASE] = {PWM_A_GPIO, PWM_B_GPIO, PWM_C_GPIO},
    [MOTOR_SIDE] = {PWM2_A_GPIO, PWM2_B_GPIO, PWM2_C_GPIO},
};

static const ledc_channel_t motor_pwm_channel[MOTOR_COUNT][3] = {
    [MOTOR_BASE] = {LEDC_CHANNEL_A, LEDC_CHANNEL_B, LEDC_CHANNEL_C},
    [MOTOR_SIDE] = {LEDC_CHANNEL_2A, LEDC_CHANNEL_2B, LEDC_CHANNEL_2C},
};

static void config_pwm_channel(gpio_num_t gpio, ledc_channel_t channel)
{
    ledc_channel_config_t ledc_channel_cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel_cfg);
}

void foc_pwm_init(void)
{
    gpio_config_t en_gpio_cfg = {
        .pin_bit_mask = 1ULL << ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_gpio_cfg);
    gpio_set_level(ENABLE_GPIO, 1);
    ESP_LOGI(TAG, "motor driver enable gpio %d set high", ENABLE_GPIO);

    ledc_timer_config_t ledc_timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer_cfg);

    for (uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        for (uint8_t phase = 0; phase < 3; phase++) {
            config_pwm_channel(motor_pwm_gpio[motor][phase], motor_pwm_channel[motor][phase]);
        }
    }

    ESP_LOGI(TAG, "PWM init done: 2 motors, 30kHz, 8bit");
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void setPhaseVoltageMotor(uint8_t motor, float Uq, float Ud, float angle_el)
{
    if (motor >= MOTOR_COUNT) return;

    angle_el = normalizeAngle(angle_el);

    (void)Ud;
    Ualpha = -Uq * sinf(angle_el);
    Ubeta = Uq * cosf(angle_el);

    Ua = Ualpha + voltage_power_supply / 2.0f;
    Ub = (sqrtf(3.0f) * Ubeta - Ualpha) / 2.0f + voltage_power_supply / 2.0f;
    Uc = (-Ualpha - sqrtf(3.0f) * Ubeta) / 2.0f + voltage_power_supply / 2.0f;

    dc_a = constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_pwm_channel[motor][0], (uint32_t)(dc_a * 255.0f));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_pwm_channel[motor][0]);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_pwm_channel[motor][1], (uint32_t)(dc_b * 255.0f));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_pwm_channel[motor][1]);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_pwm_channel[motor][2], (uint32_t)(dc_c * 255.0f));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_pwm_channel[motor][2]);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    setPhaseVoltageMotor(MOTOR_BASE, Uq, Ud, angle_el);
}
