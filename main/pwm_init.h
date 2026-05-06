#ifndef __PWM_INIT_H
#define __PWM_INIT_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

void foc_pwm_init(void);

#define MOTOR_BASE          0
#define MOTOR_SIDE          1
#define MOTOR_COUNT         2

#define PWM_A_GPIO          GPIO_NUM_32
#define PWM_B_GPIO          GPIO_NUM_33
#define PWM_C_GPIO          GPIO_NUM_25
#define ENABLE_GPIO         GPIO_NUM_12
#define I2C_SDA_GPIO        GPIO_NUM_19
#define I2C_SCL_GPIO        GPIO_NUM_18

#define PWM2_A_GPIO         GPIO_NUM_14
#define PWM2_B_GPIO         GPIO_NUM_26
#define PWM2_C_GPIO         GPIO_NUM_27
#define I2C2_SDA_GPIO       GPIO_NUM_23
#define I2C2_SCL_GPIO       GPIO_NUM_5

#define PI_UART_TX_GPIO     GPIO_NUM_17
#define PI_UART_RX_GPIO     GPIO_NUM_16

#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_CHANNEL_A    LEDC_CHANNEL_0
#define LEDC_CHANNEL_B    LEDC_CHANNEL_1
#define LEDC_CHANNEL_C    LEDC_CHANNEL_2
#define LEDC_CHANNEL_2A   LEDC_CHANNEL_3
#define LEDC_CHANNEL_2B   LEDC_CHANNEL_4
#define LEDC_CHANNEL_2C   LEDC_CHANNEL_5
#define LEDC_FREQ         30000
#define LEDC_RESOLUTION   LEDC_TIMER_8_BIT

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.14159265358979323846f
#define POLE_PAIRS        7

#define electricalAngle(angle) (angle * POLE_PAIRS)
#define normalizeAngle(angle)   ((fmod(angle, 2*PI)>=0)?(fmod(angle, 2*PI)):(fmod(angle, 2*PI)+2*PI))

void setPhaseVoltage(float Uq,float Ud, float angle_el);
void setPhaseVoltageMotor(uint8_t motor, float Uq, float Ud, float angle_el);
extern float voltage_power_supply;
extern float Ualpha, Ubeta, Ua, Ub, Uc, dc_a, dc_b, dc_c;
extern float zero_electric_angle;
extern float zero_electric_angle_m1;
 
#endif
