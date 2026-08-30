#include "HostProtocol.h"

#include "MotorBus.h"
#include "Servo.h"
#include "usart.h"

#define HOST_RX_SIZE       128U
#define HOST_SOF0           0xA5U
#define HOST_SOF1           0x5AU
#define HOST_STATUS_PERIOD 100U

#define HOST_CMD_TARGET    0x01U
#define HOST_CMD_ENABLE    0x02U
#define HOST_CMD_ZERO      0x03U
#define HOST_CMD_STOP      0x04U
#define HOST_CMD_STATUS    0x05U
#define HOST_CMD_CLOG      0x06U
#define HOST_CMD_VELOCITY  0x07U
#define HOST_CMD_SERVO     0x08U
#define HOST_CMD_ACK       0x80U
#define HOST_CMD_STATUS_R  0x81U

static uint8_t rx_dma[HOST_RX_SIZE];
static uint16_t rx_read;
static uint8_t frame[64];
static uint8_t frame_count;
static uint8_t frame_length;
static uint32_t next_status_tick;

static uint8_t HostChecksum(const uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint8_t value = 0U;
    for (i = 0U; i < length; ++i) value ^= data[i];
    return value;
}

static void HostSend(uint8_t type, const uint8_t *payload, uint8_t length)
{
    uint8_t tx[64];
    uint8_t i;

    if ((uint16_t)length + 5U > sizeof(tx)) return;
    tx[0] = HOST_SOF0;
    tx[1] = HOST_SOF1;
    tx[2] = type;
    tx[3] = length;
    for (i = 0U; i < length; ++i) tx[4U + i] = payload[i];
    tx[4U + length] = HostChecksum(&tx[2], (uint8_t)(length + 2U));
    (void)HAL_UART_Transmit(&huart2, tx, (uint16_t)(length + 5U), 20U);
}

static void HostSendAck(uint8_t command, uint8_t axis, uint8_t result)
{
    uint8_t payload[3] = {command, axis, result};
    HostSend(HOST_CMD_ACK, payload, 3U);
}

static void HostSendStatus(void)
{
    uint8_t payload[40];
    uint8_t axis;
    uint8_t *p = payload;
    uint32_t value;

    *p++ = MotorBus_GetOnlineMask();
    *p++ = MotorBus_GetEnabledMask();
    *p++ = MotorBus_GetZeroMask();
    *p++ = MotorBus_GetStopMask();
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        const MotorState_t *state = MotorBus_GetState(axis);
        int16_t angle = (state != 0) ? state->actual_angle_tenths : 0;
        int16_t rpm = (state != 0) ? state->actual_rpm : 0;
        *p++ = (uint8_t)((uint16_t)angle >> 8);
        *p++ = (uint8_t)angle;
        *p++ = (uint8_t)((uint16_t)rpm >> 8);
        *p++ = (uint8_t)rpm;
        *p++ = (state != 0) ? state->flags : 0U;
        value = (state != 0) ? state->comm_errors : 0UL;
        *p++ = (uint8_t)(value >> 24);
        *p++ = (uint8_t)(value >> 16);
        *p++ = (uint8_t)(value >> 8);
        *p++ = (uint8_t)value;
    }
    HostSend(HOST_CMD_STATUS_R, payload, (uint8_t)(p - payload));
}

static uint8_t HostAxisOnline(uint8_t axis)
{
    return (uint8_t)((MotorBus_GetOnlineMask() & (uint8_t)(1U << axis)) != 0U);
}

static uint8_t HostAxisZeroed(uint8_t axis)
{
    return (uint8_t)((MotorBus_GetZeroMask() & (uint8_t)(1U << axis)) != 0U);
}

static void HostHandleFrame(uint8_t command, const uint8_t *data, uint8_t length)
{
    uint8_t axis;
    uint8_t result = MOTORBUS_OK;
    int16_t angle;
    uint16_t speed;

    switch (command) {
    case HOST_CMD_TARGET:
        if (length != 6U) return;
        axis = data[0];
        angle = (int16_t)(((uint16_t)data[1] << 8) | data[2]);
        speed = (uint16_t)(((uint16_t)data[3] << 8) | data[4]);
        MotorBus_SetSpeed(axis, speed);
        MotorBus_SetAccel(axis, data[5]);
        result = (uint8_t)MotorBus_RequestAngle(axis, angle);
        HostSendAck(command, axis, result);
        break;
    case HOST_CMD_ENABLE:
        if (length != 2U) return;
        axis = data[0];
        if (axis == 0xFFU) {
            if ((data[1] != 0U) && (MotorBus_GetOnlineMask() != 0x0FU)) result = MOTORBUS_REJECT_OFFLINE;
            else if ((data[1] != 0U) && (MotorBus_GetZeroMask() != 0x0FU)) result = MOTORBUS_REJECT_ZERO;
            else MotorBus_RequestEnableAll(data[1]);
        } else if (axis >= MOTORBUS_AXIS_COUNT) result = MOTORBUS_REJECT_AXIS;
        else if ((data[1] != 0U) && (HostAxisOnline(axis) == 0U)) result = MOTORBUS_REJECT_OFFLINE;
        else if ((data[1] != 0U) && (HostAxisZeroed(axis) == 0U)) result = MOTORBUS_REJECT_ZERO;
        else MotorBus_RequestEnable(axis, data[1]);
        HostSendAck(command, axis, result);
        break;
    case HOST_CMD_ZERO:
        if (length != 1U) return;
        axis = data[0];
        if (axis == 0xFFU) {
            if (MotorBus_GetOnlineMask() != 0x0FU) result = MOTORBUS_REJECT_OFFLINE;
            else MotorBus_RequestZeroAll();
        } else if (axis >= MOTORBUS_AXIS_COUNT) result = MOTORBUS_REJECT_AXIS;
        else if (HostAxisOnline(axis) == 0U) result = MOTORBUS_REJECT_OFFLINE;
        else MotorBus_RequestZero(axis);
        HostSendAck(command, axis, result);
        break;
    case HOST_CMD_STOP:
        if (length != 1U) return;
        axis = data[0];
        if (axis == 0xFFU) MotorBus_RequestStopAll();
        else if (axis >= MOTORBUS_AXIS_COUNT) result = MOTORBUS_REJECT_AXIS;
        else MotorBus_RequestStop(axis);
        HostSendAck(command, axis, result);
        break;
    case HOST_CMD_STATUS:
        if (length == 0U) HostSendStatus();
        break;
    case HOST_CMD_CLOG:
        if (length != 1U) return;
        axis = data[0];
        if (axis == 0xFFU) MotorBus_RequestResetClogAll();
        else if (axis >= MOTORBUS_AXIS_COUNT) result = MOTORBUS_REJECT_AXIS;
        else MotorBus_RequestResetClog(axis);
        HostSendAck(command, axis, result);
        break;
    case HOST_CMD_VELOCITY:
        if (length != 5U) return;
        axis = data[0];
        if (axis >= MOTORBUS_AXIS_COUNT) result = MOTORBUS_REJECT_AXIS;
        else result = (uint8_t)MotorBus_RequestVelocity(
            axis, data[1], (uint16_t)(((uint16_t)data[2] << 8) | data[3]));
        HostSendAck(command, axis, result);
        break;
    case HOST_CMD_SERVO:
        if (length != 1U) return;
        Servo_SetAngle(data[0]);
        HostSendAck(command, 0xFEU, MOTORBUS_OK);
        break;
    default:
        break;
    }
}

static void HostPushByte(uint8_t value)
{
    if (frame_count == 0U) {
        if (value == HOST_SOF0) frame[frame_count++] = value;
        return;
    }
    if (frame_count == 1U) {
        if (value == HOST_SOF1) frame[frame_count++] = value;
        else frame_count = 0U;
        return;
    }
    if (frame_count == 2U) {
        frame[frame_count++] = value;
        return;
    }
    if (frame_count == 3U) {
        frame_length = value;
        if (frame_length > 58U) { frame_count = 0U; return; }
        frame[frame_count++] = value;
        return;
    }
    frame[frame_count++] = value;
    if (frame_count == (uint8_t)(frame_length + 5U)) {
        if (HostChecksum(&frame[2], (uint8_t)(frame_length + 2U)) == frame[frame_count - 1U]) {
            HostHandleFrame(frame[2], &frame[4], frame_length);
        }
        frame_count = 0U;
    }
}

void HostProtocol_Init(void)
{
    rx_read = 0U;
    frame_count = 0U;
    next_status_tick = HAL_GetTick() + HOST_STATUS_PERIOD;
    if (HAL_UART_Receive_DMA(&huart2, rx_dma, HOST_RX_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_TC);
    }
}

void HostProtocol_Process(void)
{
    uint16_t write_index;
    uint32_t now = HAL_GetTick();

    if (huart2.hdmarx != 0) {
        write_index = (uint16_t)(HOST_RX_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
        if (write_index >= HOST_RX_SIZE) write_index = 0U;
        while (rx_read != write_index) {
            HostPushByte(rx_dma[rx_read]);
            rx_read = (uint16_t)((rx_read + 1U) % HOST_RX_SIZE);
        }
    }
    if ((int32_t)(now - next_status_tick) >= 0) {
        next_status_tick = now + HOST_STATUS_PERIOD;
        HostSendStatus();
    }
}
