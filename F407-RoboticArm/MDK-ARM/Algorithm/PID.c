#include "PID.h"

/**
 * @brief  初始化 PID 参数和历史状态。
 * @param  pid      PID 结构体指针
 * @param  kp       比例系数，越大响应越快，但过大容易抖
 * @param  ki       积分系数，用来消除长期偏差，过大容易慢慢“冲过头”
 * @param  kd       微分系数，用来抑制变化趋势；当前沿用旧工程设为 0
 * @param  max_out  输出限幅；当前云台工程中单位为 RPM
 * @param  max_int  积分限幅，避免积分饱和
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->max_output = max_out;
    pid->max_integral = max_int;

    PID_Reset(pid);
}

/**
 * @brief  清空 PID 历史状态。
 * @note   目标丢失时必须清空积分，否则重新识别到目标时云台可能突然猛转。
 */
void PID_Reset(PID_t *pid)
{
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  位置式 PID 计算。
 * @param  target   期望值，本工程中传 0，表示希望误差为 0
 * @param  current  当前误差，单位是像素
 * @retval PID 输出；单位由调用者定义，当前云台工程为 RPM
 */
float PID_Calculate(PID_t *pid, float target, float current)
{
    pid->target = target;
    pid->current = current;

    pid->error = pid->target - pid->current;

    pid->integral += pid->error;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    pid->output = (pid->Kp * pid->error) +
                  (pid->Ki * pid->integral) +
                  (pid->Kd * (pid->error - pid->last_error));

    pid->last_error = pid->error;

    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    return pid->output;
}
