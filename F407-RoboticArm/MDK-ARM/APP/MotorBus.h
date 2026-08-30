#ifndef __MOTOR_BUS_H
#define __MOTOR_BUS_H

#include "main.h"

/*
 * 四轴地址映射固定为：axis 0~3 <=> 驱动器地址 1~4。
 * 角度单位统一为 0.1°，速度单位为 RPM，加速度使用 Emm 协议 0~255。
 */
#define MOTORBUS_AXIS_COUNT             4U
#define MOTORBUS_MIN_ANGLE_TENTHS      (-1800)
#define MOTORBUS_MAX_ANGLE_TENTHS       1800
#define MOTORBUS_DEFAULT_SPEED_RPM      30U
#define MOTORBUS_DEFAULT_ACCEL          20U

typedef struct {
    int16_t target_angle_tenths;        /* 最新目标输出角度，单位 0.1°。 */
    uint16_t speed_rpm;                 /* 下一条位置命令使用的速度。 */
    uint8_t accel;                      /* 下一条位置命令使用的加速度。 */
} MotorProfile_t;

typedef struct {
    uint8_t online;                     /* 最近轮询周期内收到合法反馈。 */
    uint8_t enabled;                    /* 驱动器状态标志或使能应答。 */
    uint8_t zero_valid;                 /* 本次上电后已人工确认并清零。 */
    uint8_t stopped;                    /* 停止锁定，拒绝新的角度动作。 */
    uint8_t flags;                      /* 0x3A 原始状态标志。 */
    int16_t actual_angle_tenths;        /* 0x36 实时位置，单位 0.1°。 */
    int16_t actual_rpm;                 /* 0x35 实时转速，带方向符号。 */
    uint32_t last_rx_tick;              /* 最近合法反馈的 HAL tick。 */
    uint32_t comm_errors;               /* 累计查询超时次数。 */
} MotorState_t;

typedef enum {
    MOTORBUS_OK = 0,
    MOTORBUS_REJECT_AXIS,
    MOTORBUS_REJECT_LIMIT,
    MOTORBUS_REJECT_OFFLINE,
    MOTORBUS_REJECT_DISABLED,
    MOTORBUS_REJECT_ZERO,
    MOTORBUS_REJECT_STOPPED,
    MOTORBUS_REJECT_BUS_FAULT
} MotorBusResult_t;

void MotorBus_Init(void);
/* 主循环服务函数：解析 DMA、处理超时、串行发送一条待处理命令或查询。 */
void MotorBus_Process(void);

/* 提交角度目标；仅在所有安全条件满足时返回 MOTORBUS_OK。 */
MotorBusResult_t MotorBus_RequestAngle(uint8_t axis, int16_t target_angle_tenths);
void MotorBus_SetSpeed(uint8_t axis, uint16_t speed_rpm);
void MotorBus_SetAccel(uint8_t axis, uint8_t accel);
MotorBusResult_t MotorBus_RequestVelocity(uint8_t axis, uint8_t direction, uint16_t speed_rpm);
void MotorBus_RequestEnable(uint8_t axis, uint8_t enable);
void MotorBus_RequestEnableAll(uint8_t enable);
void MotorBus_RequestStop(uint8_t axis);
void MotorBus_RequestStopAll(void);
void MotorBus_RequestZero(uint8_t axis);
void MotorBus_RequestZeroAll(void);
void MotorBus_RequestResetClog(uint8_t axis);
void MotorBus_RequestResetClogAll(void);

const MotorProfile_t *MotorBus_GetProfile(uint8_t axis);
const MotorState_t *MotorBus_GetState(uint8_t axis);
uint8_t MotorBus_GetOnlineMask(void);
uint8_t MotorBus_GetEnabledMask(void);
uint8_t MotorBus_GetZeroMask(void);
uint8_t MotorBus_GetStopMask(void);
uint8_t MotorBus_IsSerialOk(void);
uint8_t MotorBus_HasFault(void);

#endif
