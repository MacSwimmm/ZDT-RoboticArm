/*
 * ArmControl.c
 * --------------------------------------------------------------------------
 * 四轴机械臂的人机交互与业务编排层。
 *
 * Encoder/Key 只产生无业务含义的输入事件，MotorBus 只管理电机通信，
 * OLED 只负责显示。本模块位于三者之间，决定“某个事件应该改变哪个参数、
 * 切换哪个页面、发出哪一种安全动作”，因此硬件驱动无需了解菜单逻辑。
 */

#include "ArmControl.h"

#include "Encoder.h"
#include "Key.h"
#include "MotorBus.h"
#include "OLED.h"

#define ARM_VIEW_UPDATE_MS              50U
#define ARM_NOTICE_MS                   1200U

static OledPage_t current_page;
static OledAdjustMode_t adjust_mode;
static uint8_t selected_axis;
static OledNotice_t notice;
static uint32_t notice_until;
static uint32_t next_view_update;

static void ArmControl_SetNotice(OledNotice_t value)
{
    notice = value;
    notice_until = HAL_GetTick() + ARM_NOTICE_MS;
}

static void ArmControl_ReportResult(MotorBusResult_t result)
{
    switch (result) {
    case MOTORBUS_OK: notice = OLED_NOTICE_NONE; break;
    case MOTORBUS_REJECT_LIMIT: ArmControl_SetNotice(OLED_NOTICE_LIMIT); break;
    case MOTORBUS_REJECT_OFFLINE: ArmControl_SetNotice(OLED_NOTICE_OFFLINE); break;
    case MOTORBUS_REJECT_DISABLED: ArmControl_SetNotice(OLED_NOTICE_DISABLED); break;
    case MOTORBUS_REJECT_ZERO: ArmControl_SetNotice(OLED_NOTICE_ZERO); break;
    case MOTORBUS_REJECT_STOPPED: ArmControl_SetNotice(OLED_NOTICE_STOPPED); break;
    default: ArmControl_SetNotice(OLED_NOTICE_BUS); break;
    }
}

static void ArmControl_HandleEncoder(uint8_t axis, int8_t delta)
{
    const MotorProfile_t *profile = MotorBus_GetProfile(axis);
    int32_t value;

    if ((profile == 0) || (delta == 0)) return;

    /* 四个编码器始终直接对应四个轴，当前 OLED 页面不会屏蔽旋转事件。 */
    selected_axis = axis;
    if (adjust_mode == OLED_MODE_ANGLE) {
        value = (int32_t)profile->target_angle_tenths + ((int32_t)delta * 10L);
        if ((value < MOTORBUS_MIN_ANGLE_TENTHS) || (value > MOTORBUS_MAX_ANGLE_TENTHS)) {
            ArmControl_SetNotice(OLED_NOTICE_LIMIT);
            return;
        }
        ArmControl_ReportResult(MotorBus_RequestAngle(axis, (int16_t)value));
    } else if (adjust_mode == OLED_MODE_SPEED) {
        value = (int32_t)profile->speed_rpm + ((int32_t)delta * 10L);
        if (value < 0L) value = 0L;
        if (value > 3000L) value = 3000L;
        MotorBus_SetSpeed(axis, (uint16_t)value);
        notice = OLED_NOTICE_NONE;
    } else {
        value = (int32_t)profile->accel + delta;
        if (value < 0L) value = 0L;
        if (value > 255L) value = 255L;
        MotorBus_SetAccel(axis, (uint8_t)value);
        notice = OLED_NOTICE_NONE;
    }
}

static void ArmControl_MovePage(int8_t delta)
{
    int8_t page = (int8_t)current_page + delta;

    if (page < (int8_t)OLED_PAGE_HOME) page = (int8_t)OLED_PAGE_MODE;
    if (page > (int8_t)OLED_PAGE_MODE) page = (int8_t)OLED_PAGE_HOME;
    current_page = (OledPage_t)page;
    if ((current_page >= OLED_PAGE_AXIS1) && (current_page <= OLED_PAGE_AXIS4)) {
        selected_axis = (uint8_t)(current_page - OLED_PAGE_AXIS1);
    }
}

static void ArmControl_HandleKeyShort(uint8_t key)
{
    if (key == 0U) {
        if (current_page == OLED_PAGE_MODE) {
            adjust_mode = (adjust_mode == OLED_MODE_ANGLE) ? OLED_MODE_ACCEL :
                          (OledAdjustMode_t)((uint8_t)adjust_mode - 1U);
        } else if (current_page == OLED_PAGE_ZERO) {
            selected_axis = (selected_axis == 0U) ? 3U : (uint8_t)(selected_axis - 1U);
        } else {
            ArmControl_MovePage(-1);
        }
    } else if (key == 1U) {
        if (current_page == OLED_PAGE_MODE) {
            adjust_mode = (OledAdjustMode_t)(((uint8_t)adjust_mode + 1U) % 3U);
        } else if (current_page == OLED_PAGE_ZERO) {
            selected_axis = (uint8_t)((selected_axis + 1U) % MOTORBUS_AXIS_COUNT);
        } else {
            ArmControl_MovePage(1);
        }
    } else if (key == 2U) {
        if (current_page == OLED_PAGE_MODE) {
            current_page = OLED_PAGE_HOME;
        } else if (current_page == OLED_PAGE_ZERO) {
            if (MotorBus_GetOnlineMask() == 0x0FU) {
                MotorBus_RequestZeroAll();
                ArmControl_SetNotice(OLED_NOTICE_ZERO_ALL);
            } else {
                ArmControl_SetNotice(OLED_NOTICE_OFFLINE);
            }
        }
    } else if (key == 3U) {
        if ((MotorBus_GetZeroMask() == 0x0FU) && (MotorBus_GetOnlineMask() == 0x0FU)) {
            MotorBus_RequestEnableAll(1U);
            ArmControl_SetNotice(OLED_NOTICE_ENABLED);
        } else {
            ArmControl_SetNotice((MotorBus_GetZeroMask() != 0x0FU) ? OLED_NOTICE_ZERO : OLED_NOTICE_OFFLINE);
        }
    } else if (key == 4U) {
        MotorBus_RequestStop(selected_axis);
        ArmControl_SetNotice(OLED_NOTICE_STOPPED);
    } else if (key == 5U) {
        const MotorState_t *state = MotorBus_GetState(selected_axis);
        if ((state != 0) && (state->online != 0U)) {
            MotorBus_RequestZero(selected_axis);
            ArmControl_SetNotice(OLED_NOTICE_ZERO);
        } else {
            ArmControl_SetNotice(OLED_NOTICE_OFFLINE);
        }
    }
}

static void ArmControl_HandleKeyLong(uint8_t key)
{
    if (key == 2U) {
        current_page = OLED_PAGE_HOME;
    } else if (key == 3U) {
        MotorBus_RequestEnableAll(0U);
        ArmControl_SetNotice(OLED_NOTICE_DISABLED_ALL);
    } else if (key == 4U) {
        MotorBus_RequestStopAll();
        ArmControl_SetNotice(OLED_NOTICE_STOPPED);
    }
}

static void ArmControl_UpdateView(void)
{
    OledView_t view;
    uint8_t axis;

    view.page = current_page;
    view.mode = adjust_mode;
    view.selected_axis = selected_axis;
    view.online_mask = MotorBus_GetOnlineMask();
    view.enabled_mask = MotorBus_GetEnabledMask();
    view.zero_mask = MotorBus_GetZeroMask();
    view.stop_mask = MotorBus_GetStopMask();
    view.serial_ok = MotorBus_IsSerialOk();
    view.notice = (uint8_t)notice;
    for (axis = 0U; axis < MOTORBUS_AXIS_COUNT; ++axis) {
        const MotorState_t *state = MotorBus_GetState(axis);
        const MotorProfile_t *profile = MotorBus_GetProfile(axis);
        view.actual_angle_tenths[axis] = state->actual_angle_tenths;
        view.target_angle_tenths[axis] = profile->target_angle_tenths;
        view.actual_rpm[axis] = state->actual_rpm;
        view.speed_rpm[axis] = profile->speed_rpm;
        view.accel[axis] = profile->accel;
        view.flags[axis] = state->flags;
    }
    /* OLED 只接收完整快照，不反向读取任何电机或菜单状态。 */
    OLED_UpdateView(&view);
}

void ArmControl_Init(void)
{
    current_page = OLED_PAGE_HOME;
    adjust_mode = OLED_MODE_ANGLE;
    selected_axis = 0U;
    notice = OLED_NOTICE_NONE;
    notice_until = 0U;
    next_view_update = 0U;
    ArmControl_UpdateView();
}

void ArmControl_Process(void)
{
    EncoderEvent_t encoder_event;
    KeyEvent_t key_event;
    uint32_t now = HAL_GetTick();

    /*
     * 编码器与独立按键各自拥有事件队列。主循环在这里统一消费事件，
     * 中断中不执行菜单切换、电机命令或 OLED 绘制等耗时业务。
     */
    while (Encoder_PollEvent(&encoder_event) != 0U) {
        if (encoder_event.type == ENCODER_EVENT_ROTATE) {
            ArmControl_HandleEncoder(encoder_event.index, encoder_event.delta);
        }
        /* ENCODER_EVENT_PRESS 已完成消抖并保留，首版暂不分配业务动作。 */
    }

    while (Key_PollEvent(&key_event) != 0U) {
        if (key_event.type == KEY_EVENT_SHORT) {
            ArmControl_HandleKeyShort(key_event.index);
        } else if (key_event.type == KEY_EVENT_LONG) {
            ArmControl_HandleKeyLong(key_event.index);
        }
    }

    if ((notice != OLED_NOTICE_NONE) && ((int32_t)(now - notice_until) >= 0)) {
        notice = OLED_NOTICE_NONE;
    }
    if ((int32_t)(now - next_view_update) >= 0) {
        next_view_update = now + ARM_VIEW_UPDATE_MS;
        ArmControl_UpdateView();
    }
}
