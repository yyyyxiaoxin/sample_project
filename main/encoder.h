#ifndef __ENCODER_H
#define __ENCODER_H

#include "driver/gpio.h"


float getAngle(void);
float getAngle_total(void);
void AS5600_init(gpio_num_t sda,gpio_num_t scl);
float LowPassFilter(float val);
float get_velocity();


#endif