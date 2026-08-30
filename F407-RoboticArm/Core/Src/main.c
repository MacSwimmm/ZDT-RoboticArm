/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ArmControl.h"
#include "HostProtocol.h"
#include "Encoder.h"
#include "Key.h"
#include "MotorBus.h"
#include "OLED.h"
#include "Servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * ===================== 硬件接线与地址设置说明 =====================
 *
 * 1. MaixCAM 接 USART2，115200，8N1：
 *    MaixCAM TX  -> F407 PA3 / USART2_RX
 *    MaixCAM RX  -> F407 PA2 / USART2_TX，可选，当前程序只需要 STM32 接收坐标
 *    MaixCAM GND -> F407 GND，必须共地
 *
 * 2. 四台张大头闭环步进电机接 USART1，115200，8N1：
 *    地址 1/2/3/4 分别对应丝杆/底座/大臂/小臂。
 *    F407 PA9/PA10 按驱动器手册要求接 TTL 或 RS485 物理层，必须共地。
 *    普通推挽 TTL TX 不能直接把多个驱动器 TX 并联；若模块不是可靠的
 *    多机 TTL 总线版本，应在此处增加 RS485 收发器或拆分总线。
 *    电机电源单独接驱动器电源输入，不要从开发板 5V/3V3 给电机供电。
 *
 * 3. 电机地址：
 *    四个地址必须预先设置为 1、2、3、4，并逐台确认波特率为 115200。
 *    新电机通常默认都是地址 1，改地址时只接一台电机。
 *    例如只接某台电机时，可以临时调用：
 *    Emm_V5_Modify_Motor_ID(1, true, 2);
 *    保存后断电重启，再把两台电机一起并联到 USART1。
 *
 * 4. 不要同时开启两台电机的自动返回/到位返回，TTL 总线上多个电机同时回包会冲突。
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /*
   * 输入驱动只负责采样、消抖和事件排队；业务动作在主循环 ArmControl 中执行。
   * 先初始化输入状态，可避免上电时把当前 GPIO 电平误判为一次旋转或按键事件。
   */
  Encoder_Init();
  Key_Init();

  /* MotorBus_Init 会把四个轴置为停止态，并排队发送四轴失能命令。 */
  MotorBus_Init();
  HostProtocol_Init();
  OLED_Init();
  ArmControl_Init();
  Servo_Init();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ArmControl_Process();
    HostProtocol_Process();
    MotorBus_Process();
    OLED_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    /*
     * TIM6 约 1 kHz：这里只做 GPIO 采样和事件入队。
     * 不在中断里发串口、不刷新 OLED，也不执行菜单业务，保证中断短小稳定。
     */
    Encoder_Scan1ms();
    Key_Scan1ms();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
