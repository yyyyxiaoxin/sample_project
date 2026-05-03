#include "pwm_init.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "math.h"
float voltage_power_supply = 11.1f;
float Ualpha = 0, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;
float zero_electric_angle = 0.0f;

#define TAG "init"

void foc_pwm_init(void)
{
    // ==================== 新增：GPIO12 初始化输出高电平 ====================
    gpio_config_t en_gpio_cfg = {
        .pin_bit_mask = 1ULL << ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT,        // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_gpio_cfg);
    gpio_set_level(ENABLE_GPIO, 1);      // 输出高电平
    ESP_LOGI(TAG, "GPIO12 已输出高电平 (使能)");

    // 配置LEDC定时器
    ledc_timer_config_t ledc_timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer_cfg);

    // 配置3路PWM通道
    ledc_channel_config_t ledc_channel_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    // 通道A
    ledc_channel_cfg.gpio_num   = PWM_A_GPIO;
    ledc_channel_cfg.channel    = LEDC_CHANNEL_A;
    ledc_channel_config(&ledc_channel_cfg);
    // 通道B
    ledc_channel_cfg.gpio_num   = PWM_B_GPIO;
    ledc_channel_cfg.channel    = LEDC_CHANNEL_B;
    ledc_channel_config(&ledc_channel_cfg);
    // 通道C
    ledc_channel_cfg.gpio_num   = PWM_C_GPIO;
    ledc_channel_cfg.channel    = LEDC_CHANNEL_C;
    ledc_channel_config(&ledc_channel_cfg);

    ESP_LOGI(TAG, "PWM初始化完成 30kHz 8bit");
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
    angle_el = normalizeAngle(angle_el + zero_electric_angle);
    Ualpha =  -Uq*sin(angle_el);
    Ubeta =   Uq*cos(angle_el);

    Ua = Ualpha + voltage_power_supply/2;
    Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
    Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;

    dc_a = constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    dc_b = constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    dc_c = constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_A, (uint32_t)(dc_a * 255));ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_A);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B, (uint32_t)(dc_b * 255));ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_C, (uint32_t)(dc_c * 255));ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_C);
}
