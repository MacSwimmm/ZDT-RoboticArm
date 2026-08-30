#include "tim.h"

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim3;

void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef config = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 84U - 1U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000U - 1U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
    config.OCMode = TIM_OCMODE_PWM1;
    config.Pulse = 1500U;
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &config, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

void MX_TIM6_Init(void)
{
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8400U - 1U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 10U - 1U;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
}
