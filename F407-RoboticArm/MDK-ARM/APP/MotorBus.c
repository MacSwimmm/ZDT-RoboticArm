/*
 * MotorBus.c
 * --------------------------------------------------------------------------
 * 四台张大头 Emm 电机的 USART1 单一总线所有者。
 *
 * 设计原则：项目中只有本模块可以调用 Emm_V5 发送函数。编码器中断、
 * 菜单和 OLED 只能提交请求或读取状态，不能直接占用 USART1。这样可以把
 * 位置命令、停止/失能命令、参数查询和应答解析统一串行化，避免多处同时
 * 发送造成 DMA 冲突或多机回包碰撞。
 *
 * 请求接口均采用“写入状态/置 pending 位”的方式快速返回，实际协议帧由
 * MotorBus_Process() 在主循环中按安全优先级发送。商家 Emm_V5.c/.h 保持
 * 原样，本文件仅负责调用和调度。
 */

#include "MotorBus.h"

#include "Emm_V5.h"
#include "usart.h"

#include <string.h>

#define MOTORBUS_RX_DMA_SIZE            128U
#define MOTORBUS_FRAME_SIZE             16U
#define MOTORBUS_QUERY_INTERVAL_MS      25U
#define MOTORBUS_REPLY_TIMEOUT_MS       60U
#define MOTORBUS_ONLINE_TIMEOUT_MS      1200U
#define MOTORBUS_MAX_QUERY_MISSES       3U
#define MOTORBUS_PULSES_PER_REV         3200UL

#define MOTOR_FLAG_ENABLED              0x01U
#define MOTOR_FLAG_STALL                0x04U
#define MOTOR_FLAG_STALL_PROTECT        0x08U

typedef enum {
    BUS_QUERY_POSITION = 0,
    BUS_QUERY_SPEED,
    BUS_QUERY_FLAGS
} BusQuery_t;

/* 期望运动参数和驱动器反馈分开保存，避免把“目标值”误当成“实际值”。 */
static MotorProfile_t motor_profile[MOTORBUS_AXIS_COUNT];
static MotorState_t motor_state[MOTORBUS_AXIS_COUNT];

/* USART1 RX DMA 使用环形缓冲区，主循环按 DMA 写指针增量解析。 */
static uint8_t rx_dma_buffer[MOTORBUS_RX_DMA_SIZE];
static uint16_t rx_read_index;
static uint8_t frame_buffer[MOTORBUS_FRAME_SIZE];
static uint8_t frame_count;
static uint8_t frame_expected;

/* 每一位对应一个轴；命令合并时同一轴只保留最新目标。 */
static uint8_t target_pending_mask;
static uint8_t enable_pending_mask;
static uint8_t disable_pending_mask;
static uint8_t stop_pending_mask;
static uint8_t zero_pending_mask;
static uint8_t requested_enable[MOTORBUS_AXIS_COUNT];

static uint8_t query_axis;
static BusQuery_t query_type;
static uint8_t query_waiting;
static uint8_t query_wait_addr;
static uint8_t query_wait_code;
static uint8_t last_query_axis;
static BusQuery_t last_query_type;
static uint8_t query_retry_count;
static uint8_t query_misses[MOTORBUS_AXIS_COUNT];
static uint32_t query_deadline;
static uint32_t next_query_tick;
static uint8_t target_commands_since_query;
static uint8_t bus_fault;
static uint32_t last_valid_frame_tick;

static uint8_t MotorBus_FrameLength(uint8_t code)
{
    /* 根据 Emm 返回功能码确定固定帧长，未知功能码不进入解析器。 */
    switch (code) {
    case 0x35U: return 6U;
    case 0x36U: return 8U;
    case 0x3AU: return 4U;
    case 0x0AU:
    case 0xF3U:
    case 0xFDU:
    case 0xFEU:
        return 4U;
    default:
        return 0U;
    }
}

static uint8_t MotorBus_NextBit(uint8_t mask)
{
    uint8_t axis;

    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        if ((mask & (1U << axis)) != 0U) {
            return axis;
        }
    }
    return MOTORBUS_AXIS_COUNT;
}

static int16_t MotorBus_PositionToTenths(uint8_t sign, uint32_t raw)
{
    /* 驱动器实时位置为一圈 0~65535，转换为 -180.0°~+180.0°。 */
    int32_t angle = (int32_t)(((raw & 0xFFFFUL) * 3600UL + 32768UL) / 65536UL);

    if (sign != 0U) {
        angle = -angle;
    }
    if (angle > MOTORBUS_MAX_ANGLE_TENTHS) {
        angle -= 3600L;
    } else if (angle < MOTORBUS_MIN_ANGLE_TENTHS) {
        angle += 3600L;
    }
    return (int16_t)angle;
}

static uint32_t MotorBus_AngleToPulses(int16_t angle_tenths)
{
    /* 固定点四舍五入换算，避免 3200/360 的整数截断不断累积误差。 */
    uint32_t magnitude = (uint32_t)((angle_tenths < 0) ? -angle_tenths : angle_tenths);

    return (magnitude * MOTORBUS_PULSES_PER_REV + 1800UL) / 3600UL;
}

static void MotorBus_MarkFrame(uint8_t axis, uint8_t code)
{
    uint32_t now = HAL_GetTick();

    motor_state[axis].online = 1U;
    motor_state[axis].last_rx_tick = now;
    query_misses[axis] = 0U;
    last_valid_frame_tick = now;
    if ((query_waiting != 0U) &&
        (query_wait_addr == (uint8_t)(axis + 1U)) &&
        (query_wait_code == code)) {
        query_waiting = 0U;
    }
}

static void MotorBus_ParseFrame(const uint8_t *frame, uint8_t length)
{
    uint8_t axis;
    uint8_t code;
    uint32_t raw;
    uint16_t speed;

    /* 帧头地址必须是 1~4，结尾校验字节必须为商家协议规定的 0x6B。 */
    if ((length < 4U) || (frame[length - 1U] != 0x6BU) ||
        (frame[0] < 1U) || (frame[0] > MOTORBUS_AXIS_COUNT)) {
        return;
    }

    axis = (uint8_t)(frame[0] - 1U);
    code = frame[1];
    MotorBus_MarkFrame(axis, code);

    if ((code == 0x36U) && (length == 8U)) {
        raw = ((uint32_t)frame[3] << 24) |
              ((uint32_t)frame[4] << 16) |
              ((uint32_t)frame[5] << 8) |
              (uint32_t)frame[6];
        motor_state[axis].actual_angle_tenths = MotorBus_PositionToTenths(frame[2], raw);
    } else if ((code == 0x35U) && (length == 6U)) {
        speed = (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
        motor_state[axis].actual_rpm = (frame[2] != 0U) ? -(int16_t)speed : (int16_t)speed;
    } else if ((code == 0x3AU) && (length == 4U)) {
        motor_state[axis].flags = frame[2];
        motor_state[axis].enabled = ((frame[2] & MOTOR_FLAG_ENABLED) != 0U) ? 1U : 0U;
        if ((frame[2] & (MOTOR_FLAG_STALL | MOTOR_FLAG_STALL_PROTECT)) != 0U) {
            motor_state[axis].stopped = 1U;
            stop_pending_mask |= (uint8_t)(1U << axis);
            target_pending_mask &= (uint8_t)~(1U << axis);
        }
    } else if ((code == 0x0AU) && (frame[2] == 0x02U)) {
        motor_state[axis].zero_valid = 1U;
        motor_profile[axis].target_angle_tenths = 0;
        motor_state[axis].actual_angle_tenths = 0;
    } else if ((code == 0xF3U) && (frame[2] == 0x02U)) {
        motor_state[axis].enabled = requested_enable[axis];
        if (requested_enable[axis] != 0U) {
            motor_state[axis].stopped = 0U;
        }
    } else if ((code == 0xFEU) && (frame[2] == 0x02U)) {
        motor_state[axis].stopped = 1U;
    }
}

static void MotorBus_PushRxByte(uint8_t value)
{
    uint8_t expected;

    /* 流式状态机先寻找合法地址，再根据第二字节功能码决定完整帧长。 */
    if (frame_count == 0U) {
        if ((value < 1U) || (value > MOTORBUS_AXIS_COUNT)) {
            return;
        }
        frame_buffer[frame_count++] = value;
        return;
    }

    if (frame_count == 1U) {
        expected = MotorBus_FrameLength(value);
        if (expected == 0U) {
            frame_count = 0U;
            frame_expected = 0U;
            MotorBus_PushRxByte(value);
            return;
        }
        frame_buffer[frame_count++] = value;
        frame_expected = expected;
        return;
    }

    if (frame_count < MOTORBUS_FRAME_SIZE) {
        frame_buffer[frame_count++] = value;
    } else {
        frame_count = 0U;
        frame_expected = 0U;
        return;
    }

    if ((frame_expected != 0U) && (frame_count >= frame_expected)) {
        MotorBus_ParseFrame(frame_buffer, frame_expected);
        frame_count = 0U;
        frame_expected = 0U;
    }
}

static void MotorBus_DrainRx(void)
{
    uint16_t write_index;

    if (huart1.hdmarx == 0) {
        return;
    }
    write_index = (uint16_t)(MOTORBUS_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (write_index >= MOTORBUS_RX_DMA_SIZE) {
        write_index = 0U;
    }
    while (rx_read_index != write_index) {
        MotorBus_PushRxByte(rx_dma_buffer[rx_read_index]);
        rx_read_index = (uint16_t)((rx_read_index + 1U) % MOTORBUS_RX_DMA_SIZE);
    }
}

static uint8_t MotorBus_TxSucceeded(void)
{
    if (g_emm_debug.last_tx_status == (uint8_t)HAL_OK) {
        return 1U;
    }

    /* 任一次底层发送失败都升级为总线故障，并优先安排四轴停止、失能。 */
    bus_fault = 1U;
    stop_pending_mask = 0x0FU;
    disable_pending_mask = 0x0FU;
    target_pending_mask = 0U;
    return 0U;
}

static void MotorBus_SendQuery(uint8_t axis, BusQuery_t type, uint8_t advance_sequence)
{
    uint8_t addr = (uint8_t)(axis + 1U);
    SysParams_t parameter;
    uint8_t code;

    if (type == BUS_QUERY_POSITION) {
        parameter = S_CPOS;
        code = 0x36U;
    } else if (type == BUS_QUERY_SPEED) {
        parameter = S_VEL;
        code = 0x35U;
    } else {
        parameter = S_FLAG;
        code = 0x3AU;
    }

    /* 每次只允许一个查询等待应答，防止四个地址同时返回而冲突。 */
    Emm_V5_Read_Sys_Params(addr, parameter);
    if (MotorBus_TxSucceeded() == 0U) {
        return;
    }

    query_waiting = 1U;
    query_wait_addr = addr;
    query_wait_code = code;
    query_deadline = HAL_GetTick() + MOTORBUS_REPLY_TIMEOUT_MS;
    last_query_axis = axis;
    last_query_type = type;

    if (advance_sequence != 0U) {
        query_retry_count = 0U;
        query_type = (BusQuery_t)((uint8_t)type + 1U);
        if (query_type > BUS_QUERY_FLAGS) {
            query_type = BUS_QUERY_POSITION;
            query_axis = (uint8_t)((axis + 1U) % MOTORBUS_AXIS_COUNT);
        }
    }
}

static void MotorBus_HandleQueryTimeout(uint32_t now)
{
    uint8_t axis;

    if ((query_waiting == 0U) || ((int32_t)(now - query_deadline) < 0)) {
        return;
    }

    /* 首次超时原查询重发一次；连续丢失后累计错误并判定该轴离线。 */
    if (query_retry_count == 0U) {
        query_retry_count = 1U;
        query_waiting = 0U;
        MotorBus_SendQuery(last_query_axis, last_query_type, 0U);
        return;
    }

    axis = (uint8_t)(query_wait_addr - 1U);
    query_waiting = 0U;
    ++motor_state[axis].comm_errors;
    if (query_misses[axis] < 255U) {
        ++query_misses[axis];
    }
    if (query_misses[axis] >= MOTORBUS_MAX_QUERY_MISSES) {
        motor_state[axis].online = 0U;
        motor_state[axis].enabled = 0U;
        motor_state[axis].stopped = 1U;
        target_pending_mask &= (uint8_t)~(1U << axis);
    }
}

static uint8_t MotorBus_ProcessPending(uint8_t allow_target)
{
    uint8_t axis;
    uint8_t bit;
    uint8_t direction;
    uint32_t pulses;

    /*
     * 安全优先级固定为：停止 > 失能 > 清零 > 使能 > 位置目标。
     * 一次调用最多发送一条命令，确保 USART1 始终只有一个发送者。
     */
    axis = MotorBus_NextBit(stop_pending_mask);
    if (axis < MOTORBUS_AXIS_COUNT) {
        bit = (uint8_t)(1U << axis);
        stop_pending_mask &= (uint8_t)~bit;
        target_pending_mask &= (uint8_t)~bit;
        motor_state[axis].stopped = 1U;
        Emm_V5_Stop_Now((uint8_t)(axis + 1U), false);
        (void)MotorBus_TxSucceeded();
        return 1U;
    }

    axis = MotorBus_NextBit(disable_pending_mask);
    if (axis < MOTORBUS_AXIS_COUNT) {
        bit = (uint8_t)(1U << axis);
        disable_pending_mask &= (uint8_t)~bit;
        target_pending_mask &= (uint8_t)~bit;
        requested_enable[axis] = 0U;
        motor_state[axis].enabled = 0U;
        Emm_V5_En_Control((uint8_t)(axis + 1U), false, false);
        (void)MotorBus_TxSucceeded();
        return 1U;
    }

    axis = MotorBus_NextBit(zero_pending_mask);
    if (axis < MOTORBUS_AXIS_COUNT) {
        bit = (uint8_t)(1U << axis);
        zero_pending_mask &= (uint8_t)~bit;
        motor_state[axis].zero_valid = 0U;
        Emm_V5_Reset_CurPos_To_Zero((uint8_t)(axis + 1U));
        (void)MotorBus_TxSucceeded();
        return 1U;
    }

    axis = MotorBus_NextBit(enable_pending_mask);
    if (axis < MOTORBUS_AXIS_COUNT) {
        bit = (uint8_t)(1U << axis);
        enable_pending_mask &= (uint8_t)~bit;
        requested_enable[axis] = 1U;
        Emm_V5_En_Control((uint8_t)(axis + 1U), true, false);
        if (MotorBus_TxSucceeded() != 0U) {
            motor_state[axis].stopped = 0U;
        }
        return 1U;
    }

    if (allow_target == 0U) {
        return 0U;
    }

    axis = MotorBus_NextBit(target_pending_mask);
    if (axis < MOTORBUS_AXIS_COUNT) {
        bit = (uint8_t)(1U << axis);
        target_pending_mask &= (uint8_t)~bit;
        direction = (motor_profile[axis].target_angle_tenths < 0) ? 1U : 0U;
        pulses = MotorBus_AngleToPulses(motor_profile[axis].target_angle_tenths);
        Emm_V5_Pos_Control((uint8_t)(axis + 1U), direction,
                           motor_profile[axis].speed_rpm,
                           motor_profile[axis].accel,
                           pulses, true, false);
        (void)MotorBus_TxSucceeded();
        return 2U;
    }

    return 0U;
}

void MotorBus_Init(void)
{
    uint8_t axis;

    /*
     * 上电不信任历史姿态：零点无效、目标角度为 0、四轴处于停止状态，
     * 并预置 disable_pending_mask，随后由主循环逐轴发送失能命令。
     */
    (void)memset(motor_state, 0, sizeof(motor_state));
    (void)memset(query_misses, 0, sizeof(query_misses));
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        motor_profile[axis].target_angle_tenths = 0;
        motor_profile[axis].speed_rpm = MOTORBUS_DEFAULT_SPEED_RPM;
        motor_profile[axis].accel = MOTORBUS_DEFAULT_ACCEL;
        motor_state[axis].stopped = 1U;
    }

    rx_read_index = 0U;
    frame_count = 0U;
    frame_expected = 0U;
    target_pending_mask = 0U;
    enable_pending_mask = 0U;
    disable_pending_mask = 0x0FU;
    stop_pending_mask = 0U;
    zero_pending_mask = 0U;
    query_axis = 0U;
    query_type = BUS_QUERY_POSITION;
    query_waiting = 0U;
    bus_fault = 0U;
    last_valid_frame_tick = 0U;
    next_query_tick = HAL_GetTick() + 100U;
    target_commands_since_query = 0U;

    if (HAL_UART_Receive_DMA(&huart1, rx_dma_buffer, MOTORBUS_RX_DMA_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
    } else {
        bus_fault = 1U;
    }
}

void MotorBus_Process(void)
{
    uint8_t axis;
    uint8_t pending_result;
    uint32_t now = HAL_GetTick();

    /* 先处理反馈和超时，再按优先级发送一条请求，整个函数不在中断中调用。 */
    MotorBus_DrainRx();
    MotorBus_HandleQueryTimeout(now);

    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        if ((motor_state[axis].online != 0U) &&
            ((now - motor_state[axis].last_rx_tick) > MOTORBUS_ONLINE_TIMEOUT_MS)) {
            motor_state[axis].online = 0U;
            motor_state[axis].enabled = 0U;
            motor_state[axis].stopped = 1U;
            target_pending_mask &= (uint8_t)~(1U << axis);
        }
    }

    if (query_waiting != 0U) {
        if ((stop_pending_mask != 0U) || (disable_pending_mask != 0U)) {
            query_waiting = 0U;
            (void)MotorBus_ProcessPending(0U);
        }
        return;
    }

    pending_result = MotorBus_ProcessPending(0U);
    if (pending_result != 0U) {
        next_query_tick = HAL_GetTick() + MOTORBUS_QUERY_INTERVAL_MS;
        return;
    }

    if (((int32_t)(now - next_query_tick) >= 0) &&
        ((target_pending_mask == 0U) || (target_commands_since_query >= 2U))) {
        next_query_tick = now + MOTORBUS_QUERY_INTERVAL_MS;
        MotorBus_SendQuery(query_axis, query_type, 1U);
        target_commands_since_query = 0U;
        return;
    }

    pending_result = MotorBus_ProcessPending(1U);
    if (pending_result == 2U) {
        if (target_commands_since_query < 255U) {
            ++target_commands_since_query;
        }
        return;
    }

    if ((int32_t)(now - next_query_tick) >= 0) {
        next_query_tick = now + MOTORBUS_QUERY_INTERVAL_MS;
        MotorBus_SendQuery(query_axis, query_type, 1U);
        target_commands_since_query = 0U;
    }
}

MotorBusResult_t MotorBus_RequestAngle(uint8_t axis, int16_t target_angle_tenths)
{
    /*
     * 位置动作的统一安全门：轴号、软限位、总线、在线、零点、使能、
     * 停止锁定全部通过后，才更新该轴最新目标并置 pending 位。
     */
    if (axis >= MOTORBUS_AXIS_COUNT) return MOTORBUS_REJECT_AXIS;
    if ((target_angle_tenths < MOTORBUS_MIN_ANGLE_TENTHS) ||
        (target_angle_tenths > MOTORBUS_MAX_ANGLE_TENTHS)) return MOTORBUS_REJECT_LIMIT;
    if (bus_fault != 0U) return MOTORBUS_REJECT_BUS_FAULT;
    if (motor_state[axis].online == 0U) return MOTORBUS_REJECT_OFFLINE;
    if (motor_state[axis].zero_valid == 0U) return MOTORBUS_REJECT_ZERO;
    if (motor_state[axis].enabled == 0U) return MOTORBUS_REJECT_DISABLED;
    if (motor_state[axis].stopped != 0U) return MOTORBUS_REJECT_STOPPED;

    motor_profile[axis].target_angle_tenths = target_angle_tenths;
    target_pending_mask |= (uint8_t)(1U << axis);
    return MOTORBUS_OK;
}

void MotorBus_SetSpeed(uint8_t axis, uint16_t speed_rpm)
{
    /* 速度是下一条位置命令的曲线参数，不会单独启动电机连续旋转。 */
    if (axis >= MOTORBUS_AXIS_COUNT) return;
    if (speed_rpm > 3000U) speed_rpm = 3000U;
    motor_profile[axis].speed_rpm = speed_rpm;
}

void MotorBus_SetAccel(uint8_t axis, uint8_t accel)
{
    /* 加速度范围由 uint8_t 自然限定为 0~255；0 表示直接启动。 */
    if (axis >= MOTORBUS_AXIS_COUNT) return;
    motor_profile[axis].accel = accel;
}

void MotorBus_RequestEnable(uint8_t axis, uint8_t enable)
{
    uint8_t bit;

    if (axis >= MOTORBUS_AXIS_COUNT) return;
    bit = (uint8_t)(1U << axis);
    if (enable != 0U) {
        disable_pending_mask &= (uint8_t)~bit;
        enable_pending_mask |= bit;
    } else {
        enable_pending_mask &= (uint8_t)~bit;
        disable_pending_mask |= bit;
        target_pending_mask &= (uint8_t)~bit;
    }
}

void MotorBus_RequestEnableAll(uint8_t enable)
{
    uint8_t axis;

    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        MotorBus_RequestEnable(axis, enable);
    }
}

void MotorBus_RequestStop(uint8_t axis)
{
    if (axis >= MOTORBUS_AXIS_COUNT) return;
    stop_pending_mask |= (uint8_t)(1U << axis);
    target_pending_mask &= (uint8_t)~(1U << axis);
    motor_state[axis].stopped = 1U;
}

void MotorBus_RequestStopAll(void)
{
    stop_pending_mask = 0x0FU;
    target_pending_mask = 0U;
    motor_state[0].stopped = 1U;
    motor_state[1].stopped = 1U;
    motor_state[2].stopped = 1U;
    motor_state[3].stopped = 1U;
}

void MotorBus_RequestZero(uint8_t axis)
{
    if (axis >= MOTORBUS_AXIS_COUNT) return;
    /* 清零前先锁定该轴；收到驱动器成功应答后才把 zero_valid 置 1。 */
    MotorBus_RequestStop(axis);
    motor_state[axis].zero_valid = 0U;
    zero_pending_mask |= (uint8_t)(1U << axis);
}

void MotorBus_RequestZeroAll(void)
{
    uint8_t axis;

    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        MotorBus_RequestZero(axis);
    }
}

const MotorProfile_t *MotorBus_GetProfile(uint8_t axis)
{
    return (axis < MOTORBUS_AXIS_COUNT) ? &motor_profile[axis] : 0;
}

const MotorState_t *MotorBus_GetState(uint8_t axis)
{
    return (axis < MOTORBUS_AXIS_COUNT) ? &motor_state[axis] : 0;
}

uint8_t MotorBus_GetOnlineMask(void)
{
    uint8_t axis;
    uint8_t mask = 0U;
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) if (motor_state[axis].online != 0U) mask |= (uint8_t)(1U << axis);
    return mask;
}

uint8_t MotorBus_GetEnabledMask(void)
{
    uint8_t axis;
    uint8_t mask = 0U;
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) if (motor_state[axis].enabled != 0U) mask |= (uint8_t)(1U << axis);
    return mask;
}

uint8_t MotorBus_GetZeroMask(void)
{
    uint8_t axis;
    uint8_t mask = 0U;
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) if (motor_state[axis].zero_valid != 0U) mask |= (uint8_t)(1U << axis);
    return mask;
}

uint8_t MotorBus_GetStopMask(void)
{
    uint8_t axis;
    uint8_t mask = 0U;
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) if (motor_state[axis].stopped != 0U) mask |= (uint8_t)(1U << axis);
    return mask;
}

uint8_t MotorBus_IsSerialOk(void)
{
    if (bus_fault != 0U) return 0U;
    if (last_valid_frame_tick == 0U) return 0U;
    return ((HAL_GetTick() - last_valid_frame_tick) <= MOTORBUS_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t MotorBus_HasFault(void)
{
    return bus_fault;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* USART1/DMA 异常属于总线级故障，下一轮主循环将执行停止和失能。 */
    if (huart->Instance != USART1) {
        return;
    }

    bus_fault = 1U;
    MotorBus_RequestStopAll();
    MotorBus_RequestEnableAll(0U);
    (void)HAL_UART_DMAStop(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    rx_read_index = 0U;
    if (HAL_UART_Receive_DMA(&huart1, rx_dma_buffer, MOTORBUS_RX_DMA_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
    }
}
