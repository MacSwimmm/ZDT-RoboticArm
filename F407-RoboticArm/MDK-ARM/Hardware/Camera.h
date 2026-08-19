#ifndef __CAMERA_H
#define __CAMERA_H

#include "main.h"
#include "usart.h"

/* MaixCAM 每帧只发类似 "150,120\n" 或 "N\n"，64 字节已经很宽裕。 */
#define CAM_RX_BUF_SIZE 64U

/* g_camera_debug.last_frame_type 的取值，便于在 Keil Watch 中判断最近一帧。 */
#define CAMERA_FRAME_NONE     0U
#define CAMERA_FRAME_COORD    1U
#define CAMERA_FRAME_LOST     2U
#define CAMERA_FRAME_INVALID  3U

/*
 * MaixCAM 串口调试信息。
 * 在 Keil 调试模式的 Watch 窗口中添加 g_camera_debug，即可观察原始帧、
 * 最近一次有效坐标、收包次数以及解析/UART 错误。
 */
typedef struct
{
    uint32_t rx_event_count;          /* USART2 收到帧并触发 IDLE 回调的次数 */
    uint32_t processed_frame_count;   /* 主循环已经处理的帧数 */
    uint32_t valid_coord_count;       /* 成功解析为 "x,y" 的帧数 */
    uint32_t lost_frame_count;        /* 收到 "N"/"n" 的帧数 */
    uint32_t parse_error_count;       /* 无法解析的帧数 */
    uint32_t uart_error_count;        /* USART2 错误回调次数 */
    uint32_t restart_ok_count;        /* 接收 DMA 成功重新启动次数 */
    uint32_t restart_error_count;     /* 接收 DMA 启动失败次数 */
    uint32_t last_rx_tick;            /* 最近一次收到数据时的 HAL tick */
    uint32_t last_process_tick;       /* 最近一次解析数据时的 HAL tick */
    uint32_t last_error_tick;         /* 最近一次 UART 错误时的 HAL tick */
    uint32_t last_uart_error_code;    /* 最近一次 USART2 ErrorCode */
    uint16_t last_rx_len;             /* 最近一帧的字节数 */
    int16_t last_x;                   /* 最近一次成功解析到的 X 坐标 */
    int16_t last_y;                   /* 最近一次成功解析到的 Y 坐标 */
    uint8_t target_found;             /* 1=最近一帧是有效坐标，0=丢失/无效 */
    uint8_t last_frame_type;          /* CAMERA_FRAME_* */
    uint8_t last_restart_status;      /* HAL_UARTEx_ReceiveToIdle_DMA 返回值 */
    char last_frame[CAM_RX_BUF_SIZE + 1U]; /* 最近收到的原始文本，以 \0 结尾 */
} Camera_Debug_t;

extern volatile Camera_Debug_t g_camera_debug;

void Camera_Init(void);
void Camera_Process(void);

#endif /* __CAMERA_H */
