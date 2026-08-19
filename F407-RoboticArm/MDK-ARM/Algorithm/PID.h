#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* 位置式 PID 控制器。
 * PID 本身不关心输出单位，单位由调用者定义。
 * 当前云台工程把输出直接定义为 RPM。
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target;       /* 目标值，例如画面中心 0 偏差 */
    float current;      /* 当前值，例如目标偏离中心的像素误差 */

    float error;        /* 当前误差 */
    float last_error;   /* 上一次误差，用于微分项 */
    float integral;     /* 误差积分，用于消除静差 */

    float max_integral; /* 积分限幅，避免目标丢失后积分越积越大 */
    float max_output;   /* 输出限幅；当前云台工程中单位为 RPM */

    float output;       /* PID 最终输出 */
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int);
float PID_Calculate(PID_t *pid, float target, float current);
void PID_Reset(PID_t *pid);

#endif /* __PID_H */
