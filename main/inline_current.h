#ifndef INLINE_CURRENT_H
#define INLINE_CURRENT_H

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

// 电流采样硬件参数（与原Arduino代码一致）
#define SHUNT_RESISTOR     0.01f    // 分流电阻 0.01Ω
#define AMP_GAIN           50.0f    // 运放增益 50倍
#define ADC_SAMPLE_NUM     10       // 采样平均次数
#define VOLTS_TO_AMPS      (1.0f / SHUNT_RESISTOR / AMP_GAIN)

// 电机0相电流ADC通道
#define MOT0_A_ADC_CHANNEL     ADC_CHANNEL_3   // GPIO39
#define MOT0_B_ADC_CHANNEL     ADC_CHANNEL_0   // GPIO36

// 电流结构体
typedef struct {
    float current_a;   // A相电流
    float current_b;   // B相电流
    float current_c;   // C相电流
} curr_sense_t;

// 初始化电流采样
void curr_sense_init(void);

// 获取相电流
void curr_sense_get_currents(curr_sense_t *cs);

#endif