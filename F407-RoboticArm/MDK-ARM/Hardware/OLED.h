#ifndef __ARM_OLED_H
#define __ARM_OLED_H

#include "main.h"

/* OLED 页面编号只描述显示内容，不包含任何按键或电机控制逻辑。 */
typedef enum {
    OLED_PAGE_HOME = 0,
    OLED_PAGE_AXIS1,
    OLED_PAGE_AXIS2,
    OLED_PAGE_AXIS3,
    OLED_PAGE_AXIS4,
    OLED_PAGE_ZERO,
    OLED_PAGE_MODE
} OledPage_t;

typedef enum {
    OLED_MODE_ANGLE = 0,
    OLED_MODE_SPEED,
    OLED_MODE_ACCEL
} OledAdjustMode_t;

typedef enum {
    OLED_NOTICE_NONE = 0,
    OLED_NOTICE_LIMIT,
    OLED_NOTICE_OFFLINE,
    OLED_NOTICE_DISABLED,
    OLED_NOTICE_ZERO,
    OLED_NOTICE_STOPPED,
    OLED_NOTICE_BUS,
    OLED_NOTICE_ZERO_ALL,
    OLED_NOTICE_ENABLED,
    OLED_NOTICE_DISABLED_ALL
} OledNotice_t;

typedef struct {
    /* ArmControl 生成的只读显示快照，OLED 模块据此重建显存。 */
    OledPage_t page;
    OledAdjustMode_t mode;
    uint8_t selected_axis;
    uint8_t online_mask;
    uint8_t enabled_mask;
    uint8_t zero_mask;
    uint8_t stop_mask;
    uint8_t serial_ok;
    uint8_t notice;
    int16_t actual_angle_tenths[4];
    int16_t target_angle_tenths[4];
    int16_t actual_rpm[4];
    uint16_t speed_rpm[4];
    uint8_t accel[4];
    uint8_t flags[4];
} OledView_t;

void OLED_Init(void);

/* 复制新的显示快照并标记为待刷新；函数本身不访问 I2C。 */
void OLED_UpdateView(const OledView_t *view);

/* 主循环非阻塞服务函数：每次最多发送一页，并负责断线恢复。 */
void OLED_Process(void);

/* 返回最近一次 OLED 探测/通信是否正常。 */
uint8_t OLED_IsAvailable(void);

#endif
