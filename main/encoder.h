#ifndef __ENCODER_H
#define __ENCODER_H

#include "driver/gpio.h"
#include "esp_err.h"


float getAngle(char motor);
float getAngle_total(char motor);
esp_err_t getAngle_total_checked(char motor, float *total_angle);
void AS5600_init(void);
float LowPassFilter(char motor, float val);
float get_velocity(char motor);


#endif
