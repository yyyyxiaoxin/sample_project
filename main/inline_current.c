#include "inline_current.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG     "curr_sense"

// ADC 句柄（与你的NTC驱动格式一致）
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_calibration = false;

// ADC零漂校准值
static float M0_offset_ia = 0.0f;
static float M0_offset_ib = 0.0f;
static float M1_offset_ia = 0.0f;
static float M1_offset_ib = 0.0f;

// 与你NTC代码完全一致的 ADC 校准初始化
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    return calibrated;
}

// 单次读取ADC电压（与你的NTC风格一致）
// static float adc_read_voltage(adc_channel_t channel)
// {
//     int raw = 0;
//     int voltage = 0;
    
//     adc_oneshot_read(adc_handle, channel, &raw);
//     if (do_calibration) {
//         adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage);
//     }
//     // mV → V
//     return (float)voltage / 1000.0f;
// }

// 多次采样取平均
// static float adc_read_avg(adc_channel_t channel, uint8_t samples)
// {
//     float sum = 0.0f;
//     for (int i = 0; i < samples; i++) {
//         sum += adc_read_voltage(channel);
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
//     return sum / samples;
// }
// 优化版：无延时、无阻塞、超高速、适合FOC电流环
static float adc_read_avg(adc_channel_t channel, uint8_t samples)
{
    // 限制最大采样次数，防止CPU占用过高
    if (samples > 8) samples = 8;

    int32_t sum_raw = 0;  // 用整数累加，更快
    int raw = 0;

    for (int i = 0; i < samples; i++) {
        // 直接读原始值，不延时、不阻塞
        adc_oneshot_read(adc_handle, channel, &raw);
        sum_raw += raw;
    }

    // 平均后再转电压
    int avg_raw = sum_raw / samples;
    int voltage = 0;

    if (do_calibration) {
        adc_cali_raw_to_voltage(adc_cali_handle, avg_raw, &voltage);
    }

    return (float)voltage / 1000.0f;
}

// 校准电流零漂（与原Arduino功能一致）
static void curr_sense_calibrate(void)
{
    ESP_LOGI(TAG, "正在校准电流零漂...");
    M0_offset_ia = adc_read_avg(MOT0_A_ADC_CHANNEL, ADC_SAMPLE_NUM);
    M0_offset_ib = adc_read_avg(MOT0_B_ADC_CHANNEL, ADC_SAMPLE_NUM);
    M1_offset_ia = adc_read_avg(MOT1_A_ADC_CHANNEL, ADC_SAMPLE_NUM);
    M1_offset_ib = adc_read_avg(MOT1_B_ADC_CHANNEL, ADC_SAMPLE_NUM);
    ESP_LOGI(TAG, "电流校准完成: A=%.3fV, B=%.3fV", M0_offset_ia, M0_offset_ib);
    ESP_LOGI(TAG, "电流校准完成: A=%.3fV, B=%.3fV", M1_offset_ia, M1_offset_ib);
}

// 初始化电流采样（完全对齐你的NTC初始化风格）
void curr_sense_init(void)
{
    // ADC1 单次模式配置
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    // 通道配置：12位，衰减12dB（与你的NTC一致）
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MOT0_A_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MOT0_B_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MOT1_A_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MOT1_B_ADC_CHANNEL, &chan_cfg));

    // ADC校准
    do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc_cali_handle);
    
    // 电流零漂校准
    curr_sense_calibrate();
}

// 获取相电流（核心函数）
void curr_sense_get_currents(char motor, curr_sense_t *cs)
{
    if (!cs) return;

    // 读取平均电压
    float volt_a = adc_read_avg((motor == 0) ? MOT0_A_ADC_CHANNEL : MOT1_A_ADC_CHANNEL, 5);
    float volt_b = adc_read_avg((motor == 0) ? MOT0_B_ADC_CHANNEL : MOT1_B_ADC_CHANNEL, 5);

    // 减去零漂 → 转换为电流
    cs->motor = motor;
    cs->current_a = (volt_a - ((motor == 0) ? M0_offset_ia : M1_offset_ia)) * VOLTS_TO_AMPS;
    cs->current_b = (volt_b - ((motor == 0) ? M0_offset_ib : M1_offset_ib)) * VOLTS_TO_AMPS;
    cs->current_c = (-1)*(cs->current_a + cs->current_b ); // 两相采样，C相为0
}
