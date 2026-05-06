#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_port_t i2c_port;
    gpio_num_t sda;
    gpio_num_t scl;
    float angle_last;
    int16_t rotation;
    int64_t velocity_last_us;
    float velocity_last_angle;
    int64_t filter_last_us;
    float filter_last_value;
} as5600_encoder_t;

extern as5600_encoder_t encoder_m0;
extern as5600_encoder_t encoder_m1;

esp_err_t AS5600_encoder_init(as5600_encoder_t *encoder, i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
float AS5600_get_angle(as5600_encoder_t *encoder);
float AS5600_get_angle_total(as5600_encoder_t *encoder);
float AS5600_get_velocity(as5600_encoder_t *encoder);
float LowPassFilterEncoder(as5600_encoder_t *encoder, float val);

void AS5600_init(gpio_num_t sda,gpio_num_t scl);
float getAngle(void);
float getAngle_total(void);
float LowPassFilter(float val);
float get_velocity();


#endif
