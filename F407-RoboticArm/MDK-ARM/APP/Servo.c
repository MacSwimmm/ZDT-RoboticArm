#include "Servo.h"
#include "tim.h"

#define SERVO_MIN_US  500U
#define SERVO_MAX_US 2500U

static uint8_t servo_angle;

void Servo_Init(void)
{
    servo_angle = 90U;
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    Servo_SetAngle(servo_angle);
}

void Servo_SetAngle(uint8_t angle)
{
    uint32_t pulse;
    if (angle > 180U) angle = 180U;
    servo_angle = angle;
    pulse = SERVO_MIN_US + ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

uint8_t Servo_GetAngle(void)
{
    return servo_angle;
}
