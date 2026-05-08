#ifndef __PWM_INIT_H
#define __PWM_INIT_H

void foc_pwm_init(void);

#define M0_PWM_A_GPIO    GPIO_NUM_32
#define M0_PWM_B_GPIO    GPIO_NUM_33
#define M0_PWM_C_GPIO    GPIO_NUM_25
#define M1_PWM_A_GPIO    GPIO_NUM_26
#define M1_PWM_B_GPIO    GPIO_NUM_27
#define M1_PWM_C_GPIO    GPIO_NUM_14


#define ENABLE_GPIO   GPIO_NUM_12

#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_CHANNEL_M0_A    LEDC_CHANNEL_0
#define LEDC_CHANNEL_M0_B    LEDC_CHANNEL_1
#define LEDC_CHANNEL_M0_C    LEDC_CHANNEL_2
#define LEDC_CHANNEL_M1_A    LEDC_CHANNEL_3
#define LEDC_CHANNEL_M1_B    LEDC_CHANNEL_4
#define LEDC_CHANNEL_M1_C    LEDC_CHANNEL_5

#define LEDC_FREQ         30000
#define LEDC_RESOLUTION   LEDC_TIMER_8_BIT

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.14159265358979323846f
#define POLE_PAIRS        7

#define electricalAngle(angle) (angle * POLE_PAIRS)
#define normalizeAngle(angle)   ((fmod(angle, 2*PI)>=0)?(fmod(angle, 2*PI)):(fmod(angle, 2*PI)+2*PI))

void setPhaseVoltage(char motor, float Uq, float Ud, float angle_el);
extern float voltage_power_supply;
extern float Ualpha, Ubeta, Ua, Ub, Uc, dc_a, dc_b, dc_c;
extern float M0_zero_electric_angle;
extern float M1_zero_electric_angle;
 
#endif