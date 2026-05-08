#include "encoder.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "math.h"

#define PI                  3.14159265f
#define TAG                 "encoder"

#define M0_I2C_SDA_GPIO    GPIO_NUM_19
#define M0_I2C_SCL_GPIO    GPIO_NUM_18
#define M1_I2C_SDA_GPIO    GPIO_NUM_23
#define M1_I2C_SCL_GPIO    GPIO_NUM_5

#define AS5600_ADDR         0x36
#define SOFT_I2C_DELAY_US   5

static inline gpio_num_t motor_sda(char motor)
{
    return (motor == 0) ? M0_I2C_SDA_GPIO : M1_I2C_SDA_GPIO;
}

static inline gpio_num_t motor_scl(char motor)
{
    return (motor == 0) ? M0_I2C_SCL_GPIO : M1_I2C_SCL_GPIO;
}

static inline void soft_i2c_delay(void)
{
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
}

static inline void soft_i2c_sda(gpio_num_t sda, int level)
{
    gpio_set_level(sda, level);
}

static inline void soft_i2c_scl(gpio_num_t scl, int level)
{
    gpio_set_level(scl, level);
}

static bool soft_i2c_wait_scl_high(gpio_num_t scl)
{
    for (int i = 0; i < 50; i++) {
        if (gpio_get_level(scl)) {
            return true;
        }
        esp_rom_delay_us(2);
    }
    return false;
}

static bool soft_i2c_clock_high(gpio_num_t scl)
{
    soft_i2c_scl(scl, 1);
    if (!soft_i2c_wait_scl_high(scl)) {
        return false;
    }
    soft_i2c_delay();
    return true;
}

static void soft_i2c_stop(gpio_num_t sda, gpio_num_t scl)
{
    soft_i2c_sda(sda, 0);
    soft_i2c_delay();
    soft_i2c_scl(scl, 1);
    soft_i2c_wait_scl_high(scl);
    soft_i2c_delay();
    soft_i2c_sda(sda, 1);
    soft_i2c_delay();
}

static bool soft_i2c_start(gpio_num_t sda, gpio_num_t scl)
{
    soft_i2c_sda(sda, 1);
    soft_i2c_scl(scl, 1);
    if (!soft_i2c_wait_scl_high(scl)) {
        return false;
    }
    soft_i2c_delay();
    soft_i2c_sda(sda, 0);
    soft_i2c_delay();
    soft_i2c_scl(scl, 0);
    soft_i2c_delay();
    return true;
}

static bool soft_i2c_write_byte(gpio_num_t sda, gpio_num_t scl, uint8_t data)
{
    for (int bit = 7; bit >= 0; bit--) {
        soft_i2c_sda(sda, (data >> bit) & 0x01);
        soft_i2c_delay();
        if (!soft_i2c_clock_high(scl)) {
            return false;
        }
        soft_i2c_scl(scl, 0);
        soft_i2c_delay();
    }

    soft_i2c_sda(sda, 1);
    soft_i2c_delay();
    if (!soft_i2c_clock_high(scl)) {
        return false;
    }
    bool ack = (gpio_get_level(sda) == 0);
    soft_i2c_scl(scl, 0);
    soft_i2c_delay();
    return ack;
}

static bool soft_i2c_read_byte(gpio_num_t sda, gpio_num_t scl, uint8_t *data, bool ack)
{
    uint8_t value = 0;
    soft_i2c_sda(sda, 1);

    for (int bit = 7; bit >= 0; bit--) {
        if (!soft_i2c_clock_high(scl)) {
            return false;
        }
        if (gpio_get_level(sda)) {
            value |= (1 << bit);
        }
        soft_i2c_scl(scl, 0);
        soft_i2c_delay();
    }

    soft_i2c_sda(sda, ack ? 0 : 1);
    soft_i2c_delay();
    if (!soft_i2c_clock_high(scl)) {
        return false;
    }
    soft_i2c_scl(scl, 0);
    soft_i2c_sda(sda, 1);
    soft_i2c_delay();

    *data = value;
    return true;
}

void AS5600_init(){
    const gpio_num_t pins[] = {
        M0_I2C_SDA_GPIO, M0_I2C_SCL_GPIO,
        M1_I2C_SDA_GPIO, M1_I2C_SCL_GPIO
    };

    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_pull_mode(pins[i], GPIO_PULLUP_ONLY);
        gpio_set_level(pins[i], 1);
    }
}

esp_err_t AS5600_read_word(char motor,char reg, uint16_t *data) {
    uint8_t recv_buf[2] = {0};
    gpio_num_t sda = motor_sda(motor);
    gpio_num_t scl = motor_scl(motor);
    esp_err_t ret = ESP_OK;

    if (!soft_i2c_start(sda, scl) ||
        !soft_i2c_write_byte(sda, scl, (AS5600_ADDR << 1) | 0) ||
        !soft_i2c_write_byte(sda, scl, (uint8_t)reg) ||
        !soft_i2c_start(sda, scl) ||
        !soft_i2c_write_byte(sda, scl, (AS5600_ADDR << 1) | 1) ||
        !soft_i2c_read_byte(sda, scl, &recv_buf[0], true) ||
        !soft_i2c_read_byte(sda, scl, &recv_buf[1], false)) {
        ret = ESP_ERR_INVALID_STATE;
    }
    soft_i2c_stop(sda, scl);

    if(ret == ESP_OK) {
        uint16_t raw = (recv_buf[0] << 8) | recv_buf[1];
        *data = raw & 0x0FFF;
    } else {
        static uint32_t fail_count[2] = {0, 0};
        int idx = (motor == 0) ? 0 : 1;
        if ((fail_count[idx]++ % 50) == 0) {
            ESP_LOGW(TAG, "AS5600 M%d read failed: %s", motor, esp_err_to_name(ret));
        }
    }
    return ret;
}
// esp_err_t AS5600_read_word(uint8_t reg,uint16_t* data)
// {
//     uint8_t buf[1] = {reg};
//     esp_err_t ret =  i2c_master_transmit_receive(AS5600_dev_handle,buf,1,(uint8_t*)data,2,500);
//     return ret;
// }
static float last_valid_angle[2] = {0.0f, 0.0f};

static esp_err_t getAngle_checked(char motor, float *angle)
{
    uint16_t angle_data = 0;
    int idx = (motor == 0) ? 0 : 1;

    esp_err_t ret = AS5600_read_word(motor, 0x0C, &angle_data);
    if (ret != ESP_OK) {
        return ret;
    }

    *angle = angle_data * 0.08789f * PI / 180.0f;
    last_valid_angle[idx] = *angle;
    return ESP_OK;
}

float getAngle(char motor) {
    uint16_t angle_data = 0;
    // 【修复4】读取失败，返回上一次有效角度，不突变
    if(AS5600_read_word(motor, 0x0C, &angle_data) != ESP_OK) {
        return last_valid_angle[(motor == 0) ? 0 : 1];
    }
    float angle = angle_data * 0.08789f * PI / 180.0f;
    last_valid_angle[(motor == 0) ? 0 : 1] = angle; // 更新有效角度
    return angle;
}

float getAngle_total(char motor) {
    int idx = (motor == 0) ? 0 : 1;
    static float angle_last[2] = {0.0f, 0.0f};
    static int16_t rotation[2] = {0, 0};

    float angle_now = getAngle(motor);

    if(fabs(angle_now - angle_last[idx]) > 0.8f * PI * 2) {
        rotation[idx] += ((angle_now - angle_last[idx]) > 0) ? -1 : 1;
    }
    angle_last[idx] = angle_now;
    return (float)rotation[idx] * PI * 2 + angle_now;
}

esp_err_t getAngle_total_checked(char motor, float *total_angle)
{
    int idx = (motor == 0) ? 0 : 1;
    static float angle_last[2] = {0.0f, 0.0f};
    static int16_t rotation[2] = {0, 0};
    static bool initialized[2] = {false, false};

    float angle_now = 0.0f;
    esp_err_t ret = getAngle_checked(motor, &angle_now);
    if (ret != ESP_OK) {
        return ret;
    }

    if (!initialized[idx]) {
        angle_last[idx] = angle_now;
        initialized[idx] = true;
    } else if(fabs(angle_now - angle_last[idx]) > 0.8f * PI * 2) {
        rotation[idx] += ((angle_now - angle_last[idx]) > 0) ? -1 : 1;
        angle_last[idx] = angle_now;
    } else {
        angle_last[idx] = angle_now;
    }

    *total_angle = (float)rotation[idx] * PI * 2 + angle_now;
    return ESP_OK;
}

float get_velocity(char motor)
{
    int idx = (motor == 0) ? 0 : 1;
    static int64_t last_us[2] = {0, 0};
    static float last_angle[2] = {0.0f, 0.0f};
    int64_t now_us = esp_timer_get_time();
    float angle = getAngle_total(motor);
    
    float Ts = (now_us - last_us[idx]) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 5e-3f; // 避免异常时间间隔导致的速度计算错误
    
    float vel = (angle - last_angle[idx]) / Ts;
    
    last_angle[idx] = angle;
    last_us[idx] = now_us;
    return vel;    
}

float LowPassFilter(char motor, float val)
{
    int idx = (motor == 0) ? 0 : 1;
    static int64_t last_us[2] = {0, 0};
    static float val_last[2] = {0.0f, 0.0f};
    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - last_us[idx]) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 5e-3f; 
    
    float alpha = 0.1f / (0.1f + Ts);
    float val_ret = alpha * val_last[idx] + (1.0f - alpha) * val;
    val_last[idx] = val_ret;
    last_us[idx] = now_us;
    return val_ret;
}

