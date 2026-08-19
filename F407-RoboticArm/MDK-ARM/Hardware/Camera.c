#include "Camera.h"
#include "Gimbal_Control.h"
#include <stdio.h>
#include <string.h>

/* USART2 接 MaixCAM：PA3=RX 接 MaixCAM TX，PA2=TX 可选接 MaixCAM RX。 */
static uint8_t cam_rx_buffer[CAM_RX_BUF_SIZE + 1U];
static volatile uint8_t cam_rx_flag = 0U;
static volatile uint16_t cam_rx_len = 0U;

/* Keil Watch 中添加 g_camera_debug，可直接观察 MaixCAM 接收与解析状态。 */
volatile Camera_Debug_t g_camera_debug = {0};

static void Camera_RestartReceive(void)
{
    HAL_StatusTypeDef status;

    /* ReceiveToIdle_DMA 会在收到一帧后遇到串口空闲(IDLE)就回调，
     * 非常适合 MaixCAM 这种 "x,y\n" 的短文本协议。
     */
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, cam_rx_buffer, CAM_RX_BUF_SIZE);
    g_camera_debug.last_restart_status = (uint8_t)status;

    if (status == HAL_OK) {
        g_camera_debug.restart_ok_count++;
        /* 半传输中断对文本帧没有意义，关掉后只在“空闲/满缓冲”时处理。 */
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    } else {
        g_camera_debug.restart_error_count++;
    }
}

/**
 * @brief 初始化视觉串口接收。
 */
void Camera_Init(void)
{
    memset((void *)&g_camera_debug, 0, sizeof(g_camera_debug));
    cam_rx_flag = 0U;
    cam_rx_len = 0U;
    memset(cam_rx_buffer, 0, sizeof(cam_rx_buffer));
    Camera_RestartReceive();
}

/**
 * @brief 解析 MaixCAM 发来的坐标帧。
 * @note  旧工程的 MaixCAM 脚本找到目标时发送 "x,y\n"，丢失目标时发送 "N\n"。
 */
void Camera_Process(void)
{
    if (cam_rx_flag == 0U) {
        return;
    }

    int16_t x = 0;
    int16_t y = 0;
    uint16_t len = cam_rx_len;
    uint16_t i;

    if (len > CAM_RX_BUF_SIZE) {
        len = CAM_RX_BUF_SIZE;
    }
    cam_rx_buffer[len] = '\0';

    for (i = 0U; i < len; i++) {
        g_camera_debug.last_frame[i] = (char)cam_rx_buffer[i];
    }
    g_camera_debug.last_frame[len] = '\0';
    g_camera_debug.last_process_tick = HAL_GetTick();
    g_camera_debug.processed_frame_count++;

    if ((cam_rx_buffer[0] == 'N') || (cam_rx_buffer[0] == 'n')) {
        /* MaixCAM 主动报告目标丢失。 */
        g_camera_debug.lost_frame_count++;
        g_camera_debug.target_found = 0U;
        g_camera_debug.last_frame_type = CAMERA_FRAME_LOST;
        Gimbal_UpdateTarget(0, 0, 0U);
    } else if (sscanf((char *)cam_rx_buffer, "%hd,%hd", &x, &y) == 2) {
        /* 成功解析到目标中心坐标。 */
        g_camera_debug.valid_coord_count++;
        g_camera_debug.last_x = x;
        g_camera_debug.last_y = y;
        g_camera_debug.target_found = 1U;
        g_camera_debug.last_frame_type = CAMERA_FRAME_COORD;
        Gimbal_UpdateTarget(x, y, 1U);
    } else {
        /* 收到乱码/半帧/未知格式时按丢失处理，保证电机不会继续沿旧方向跑。 */
        g_camera_debug.parse_error_count++;
        g_camera_debug.target_found = 0U;
        g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
        Gimbal_UpdateTarget(0, 0, 0U);
    }

    cam_rx_flag = 0U;
    Camera_RestartReceive();
}

/**
 * @brief HAL 串口空闲接收回调。
 * @note  USART2 专门给 MaixCAM；USART1 是电机总线，这里不要处理 USART1。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        cam_rx_len = Size;
        cam_rx_flag = 1U;
        g_camera_debug.rx_event_count++;
        g_camera_debug.last_rx_len = Size;
        g_camera_debug.last_rx_tick = HAL_GetTick();

        /* 先停 DMA，等主循环解析完这帧再重启，避免缓冲区一边写一边读。 */
        HAL_UART_DMAStop(&huart2);
    }
}

/**
 * @brief 串口错误恢复。
 * @note  MaixCAM 连续发送时，如果偶发 ORE/FE/NE 错误，清掉错误并重新开 DMA。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        g_camera_debug.uart_error_count++;
        g_camera_debug.last_uart_error_code = huart->ErrorCode;
        g_camera_debug.last_error_tick = HAL_GetTick();
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        Camera_RestartReceive();
    }
}
