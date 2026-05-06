#include "encoder.h"

#include <math.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

#define PI 3.14159265f

as5600_encoder_t encoder_m0 = {0};
as5600_encoder_t encoder_m1 = {0};

esp_err_t AS5600_encoder_init(as5600_encoder_t *encoder, i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    if (!encoder) return ESP_ERR_INVALID_ARG;

    *encoder = (as5600_encoder_t) {
        .i2c_port = port,
        .sda = sda,
        .scl = scl,
    };

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .i2c_port = port,
        .flags.enable_internal_pullup = GPIO_PULLUP_ENABLE,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &encoder->bus_handle);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(encoder->bus_handle, &dev_cfg, &encoder->dev_handle);
}

void AS5600_init(gpio_num_t sda, gpio_num_t scl)
{
    AS5600_encoder_init(&encoder_m0, I2C_NUM_0, sda, scl);
}

static esp_err_t AS5600_read_word(as5600_encoder_t *encoder, uint8_t reg, uint16_t *data)
{
    if (!encoder || !encoder->dev_handle || !data) return ESP_ERR_INVALID_ARG;

    uint8_t buf[1] = {reg};
    uint8_t recv_buf[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(
        encoder->dev_handle,
        buf, 1,
        recv_buf, 2,
        500
    );

    if (ret == ESP_OK) {
        uint16_t raw = (recv_buf[0] << 8) | recv_buf[1];
        *data = raw & 0x0FFF;
    }
    return ret;
}

float AS5600_get_angle(as5600_encoder_t *encoder)
{
    uint16_t angle_data = 0;
    AS5600_read_word(encoder, 0x0C, &angle_data);
    return angle_data * 0.08789f * PI / 180.0f;
}

float AS5600_get_angle_total(as5600_encoder_t *encoder)
{
    if (!encoder) return 0.0f;

    float angle_now = AS5600_get_angle(encoder);
    if (fabsf(angle_now - encoder->angle_last) > 0.8f * PI * 2.0f) {
        encoder->rotation += ((angle_now - encoder->angle_last) > 0.0f) ? -1 : 1;
    }
    encoder->angle_last = angle_now;

    return (float)encoder->rotation * PI * 2.0f + angle_now;
}

float AS5600_get_velocity(as5600_encoder_t *encoder)
{
    if (!encoder) return 0.0f;

    int64_t now_us = esp_timer_get_time();
    float angle = AS5600_get_angle_total(encoder);
    float Ts = (now_us - encoder->velocity_last_us) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 5e-3f;

    float vel = (angle - encoder->velocity_last_angle) / Ts;
    encoder->velocity_last_angle = angle;
    encoder->velocity_last_us = now_us;
    return vel;
}

float LowPassFilterEncoder(as5600_encoder_t *encoder, float val)
{
    if (!encoder) return val;

    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - encoder->filter_last_us) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 5e-3f;

    float alpha = 0.1f / (0.1f + Ts);
    float val_ret = alpha * encoder->filter_last_value + (1.0f - alpha) * val;
    encoder->filter_last_value = val_ret;
    encoder->filter_last_us = now_us;
    return val_ret;
}

float getAngle(void)
{
    return AS5600_get_angle(&encoder_m0);
}

float getAngle_total(void)
{
    return AS5600_get_angle_total(&encoder_m0);
}

float get_velocity(void)
{
    return AS5600_get_velocity(&encoder_m0);
}

float LowPassFilter(float val)
{
    return LowPassFilterEncoder(&encoder_m0, val);
}
