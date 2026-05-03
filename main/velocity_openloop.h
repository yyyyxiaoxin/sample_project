#ifndef __VELOCITY_OPENLOOP_H
#define __VELOCITY_OPENLOOP_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float limit;
    float ramp;
} PIDController;

typedef enum {
    PID_CURRENT,    // 对应 current_pid
    PID_SITE,       // 对应 site_pid
    PID_VELOCITY,   // 对应 velocity_pid
    PID_SITE0,
    PID_VELOCITY1,
    PID_MAX_COUNT   // 统计总数
} PIDName;

// 函数声明
float velocityOpenloop(float target_v);
extern float openloop_voltage;
float PID_Controller(float error, PIDController *pid);

#endif