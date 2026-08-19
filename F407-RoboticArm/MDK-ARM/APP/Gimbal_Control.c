#include "Gimbal_Control.h"
#include "Emm_V5.h"
#include "PID.h"
#include "gpio.h"
#include "usart.h"

/* ===================== 用户需要最常改的参数 ===================== */

/* 两台张大头电机的串口地址，X=1，Y=2。 */
#define ZDT_MOTOR_X_ADDR                 1U
#define ZDT_MOTOR_Y_ADDR                 2U

/* 追踪阶段先使用低速上限。方向和机械安全验证后再逐步调大。 */
#define ZDT_MAX_RPM                      20U
#define ZDT_MIN_TRACK_RPM                2U

/* Y 轴上电前需要放在机械中心。两侧各80度作为无编码器反馈时的保守软限位。 */
#define Y_SOFT_LIMIT_DEG                 80.0f

/* 正常追踪的速度模式加速度。0表示直接启动，数值越大加速越快。 */
#define ZDT_TRACK_ACC                    20U

/* PID 输出直接以 RPM 为单位。速度闭环做位置跟随时 P 项已足够，
 * 调通方向前关闭 I/D，避免正反馈时积分继续加速。
 */
#define PID_KP_RPM                       0.15f
#define PID_KI_RPM                       0.0f
#define PID_KD_RPM                       0.0f
#define PID_MAX_OUTPUT_RPM               20.0f
#define PID_MAX_INTEGRAL                 0.0f

/* 逻辑正/负速度到驱动器 DIR 位的初始映射。不直接猜测安装方向，
 * 上电后的低速方向自检会实际观察坐标是否靠近中心，必要时只反转对应轴。
 */
#define ZDT_X_DIR_POSITIVE               1U
#define ZDT_X_DIR_NEGATIVE               0U
#define ZDT_Y_DIR_POSITIVE               1U
#define ZDT_Y_DIR_NEGATIVE               0U

/* 方向自检：目标需要在待测轴上偏离中心至少25像素。每次只动一个轴，
 * 5RPM 运行150ms后立即用0RPM停止，再比较误差。
 */
#define GIMBAL_AUTO_DIRECTION_VERIFY     1U
#define DIRECTION_TEST_RPM               5
#define DIRECTION_TEST_PREPARE_MS        80U
#define DIRECTION_TEST_RUN_MS            150U
#define DIRECTION_TEST_SETTLE_MS         80U
#define DIRECTION_TEST_MIN_ERROR_PX      25.0f
#define DIRECTION_TEST_MIN_CHANGE_PX     3.0f
#define DIRECTION_TEST_STABLE_CHANGE_PX  4
#define DIRECTION_TEST_PREPARE_SAMPLES   2U
#define DIRECTION_TEST_MIN_SAMPLES       3U
#define DIRECTION_TEST_MAX_INCONCLUSIVE  3U

/* PA1 板载 LED 的亮灭电平。 */
#define LED_ON_LEVEL                     GPIO_PIN_SET
#define LED_OFF_LEVEL                    GPIO_PIN_RESET

#define GIMBAL_AXIS_NONE                 0U
#define GIMBAL_AXIS_X                    1U
#define GIMBAL_AXIS_Y                    2U

#define GIMBAL_FAULT_NONE                0U
#define GIMBAL_FAULT_BOTH_DIRECTIONS     1U
#define GIMBAL_FAULT_NO_MOVEMENT         2U
#define GIMBAL_FAULT_MOTOR_INIT          3U

/* ===================== 内部状态 ===================== */

PID_t PID_X;
PID_t PID_Y;
volatile GimbalDebug_t g_gimbal_debug = {0};

static int16_t target_x = 0;
static int16_t target_y = 0;
static uint8_t target_found = 0U;

static float filtered_x = CAM_CENTER_X;
static float filtered_y = CAM_CENTER_Y;
#define EMA_ALPHA                        0.4f

static uint32_t last_update_time = 0U;
static uint32_t valid_target_sample_count = 0U;
static int32_t last_speed_x = 0;
static int32_t last_speed_y = 0;
static float y_estimated_angle_deg = 0.0f;
static uint32_t y_angle_update_tick = 0U;
static uint8_t tracking_was_active = 0U;
static uint8_t motor_rx_discard[64];

static uint8_t direction_verified_x = 0U;
static uint8_t direction_verified_y = 0U;
static uint8_t direction_inverted_x = 0U;
static uint8_t direction_inverted_y = 0U;
static uint8_t direction_flip_used_x = 0U;
static uint8_t direction_flip_used_y = 0U;
static uint8_t direction_inconclusive_x = 0U;
static uint8_t direction_inconclusive_y = 0U;
static uint8_t direction_test_axis = GIMBAL_AXIS_NONE;
static uint32_t direction_state_tick = 0U;
static uint32_t direction_prepare_sample_count = 0U;
static int16_t direction_prepare_coord = 0;
static uint32_t direction_test_start_sample_count = 0U;
static float direction_test_start_abs_error = 0.0f;

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int32_t AbsInt32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

/**
 * @brief 等待 USART1 上一次 DMA 发送结束。
 * @note  Emm_V5 的发送包装层还会在物理发送完成后保留10ms帧间隔。
 */
static HAL_StatusTypeDef MotorBus_WaitTxReady(void)
{
    uint32_t start_tick = HAL_GetTick();

    while (huart1.gState != HAL_UART_STATE_READY) {
        if ((HAL_GetTick() - start_tick) > 5U) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static uint8_t MotorBus_RecordLastResult(void)
{
    HAL_StatusTypeDef status = (HAL_StatusTypeDef)g_emm_debug.last_tx_status;

    if ((status == HAL_OK) && (MotorBus_WaitTxReady() != HAL_OK)) {
        status = HAL_TIMEOUT;
    }

    g_gimbal_debug.last_motor_tx_status = (uint8_t)status;
    return (status == HAL_OK) ? 1U : 0U;
}

static void MotorBus_StartReplyDrain(void)
{
    /* 不解析电机回包，仅用循环 RX DMA 排空 USART1，避免 ORE。 */
    if (HAL_UART_Receive_DMA(&huart1, motor_rx_discard, sizeof(motor_rx_discard)) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
    }
}

static void Led_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, state);
}

static void Led_TrackingBlink(void)
{
    if (((HAL_GetTick() / 100U) % 2U) == 0U) {
        Led_Set(LED_ON_LEVEL);
    } else {
        Led_Set(LED_OFF_LEVEL);
    }
}

static uint16_t Limit_Rpm_Command(int32_t speed_rpm)
{
    uint32_t rpm = (uint32_t)AbsInt32(speed_rpm);

    if (rpm > ZDT_MAX_RPM) {
        rpm = ZDT_MAX_RPM;
    }

    return (uint16_t)rpm;
}

static int32_t PID_OutputToRpm(float output_rpm)
{
    int32_t rpm;

    if (output_rpm > 0.0f) {
        rpm = (int32_t)(output_rpm + 0.5f);
        if (rpm < (int32_t)ZDT_MIN_TRACK_RPM) {
            rpm = (int32_t)ZDT_MIN_TRACK_RPM;
        }
    } else if (output_rpm < 0.0f) {
        rpm = (int32_t)(output_rpm - 0.5f);
        if (rpm > -(int32_t)ZDT_MIN_TRACK_RPM) {
            rpm = -(int32_t)ZDT_MIN_TRACK_RPM;
        }
    } else {
        rpm = 0;
    }

    if (rpm > (int32_t)ZDT_MAX_RPM) {
        rpm = (int32_t)ZDT_MAX_RPM;
    } else if (rpm < -(int32_t)ZDT_MAX_RPM) {
        rpm = -(int32_t)ZDT_MAX_RPM;
    }

    return rpm;
}

/**
 * @brief 发送一条速度命令。停止也使用已验证有效的8字节 F6/0RPM 帧。
 */
static HAL_StatusTypeDef ZDT_TransmitSpeed(uint8_t addr,
                                           int32_t speed_rpm,
                                           uint8_t pos_dir,
                                           uint8_t neg_dir,
                                           uint8_t acc)
{
    HAL_StatusTypeDef status;
    uint8_t dir = (speed_rpm >= 0) ? pos_dir : neg_dir;
    uint16_t rpm = Limit_Rpm_Command(speed_rpm);

    status = MotorBus_WaitTxReady();
    if (status != HAL_OK) {
        return status;
    }

    Emm_V5_Vel_Control(addr, dir, rpm, acc, false);
    status = (HAL_StatusTypeDef)g_emm_debug.last_tx_status;

    if ((status == HAL_OK) && (MotorBus_WaitTxReady() != HAL_OK)) {
        status = HAL_TIMEOUT;
    }

    return status;
}

static HAL_StatusTypeDef Axis_SendSpeed(uint8_t axis, int32_t speed_rpm, uint8_t acc)
{
    HAL_StatusTypeDef status;
    uint8_t addr;
    uint8_t pos_dir;
    uint8_t neg_dir;
    uint8_t inverted;

    if (axis == GIMBAL_AXIS_X) {
        addr = ZDT_MOTOR_X_ADDR;
        pos_dir = ZDT_X_DIR_POSITIVE;
        neg_dir = ZDT_X_DIR_NEGATIVE;
        inverted = direction_inverted_x;
    } else {
        addr = ZDT_MOTOR_Y_ADDR;
        pos_dir = ZDT_Y_DIR_POSITIVE;
        neg_dir = ZDT_Y_DIR_NEGATIVE;
        inverted = direction_inverted_y;
    }

    if (inverted != 0U) {
        uint8_t temp = pos_dir;
        pos_dir = neg_dir;
        neg_dir = temp;
    }

    status = ZDT_TransmitSpeed(addr, speed_rpm, pos_dir, neg_dir, acc);
    g_gimbal_debug.last_motor_tx_status = (uint8_t)status;

    if (axis == GIMBAL_AXIS_X) {
        g_gimbal_debug.last_motor_tx_status_x = (uint8_t)status;
    } else {
        g_gimbal_debug.last_motor_tx_status_y = (uint8_t)status;
    }

    /* 只有确认 HAL 已接受且发完命令后才更新软件速度。
     * 发送失败时保留旧值，下一个控制周期会自动重试。
     */
    if (status == HAL_OK) {
        if (axis == GIMBAL_AXIS_X) {
            last_speed_x = speed_rpm;
            g_gimbal_debug.commanded_rpm_x = speed_rpm;
        } else {
            last_speed_y = speed_rpm;
            g_gimbal_debug.commanded_rpm_y = speed_rpm;
        }
    }

    return status;
}

static uint8_t ZDT_ForceStopBoth(void)
{
    HAL_StatusTypeDef status_x;
    HAL_StatusTypeDef status_y;

    status_x = Axis_SendSpeed(GIMBAL_AXIS_X, 0, 0U);
    status_y = Axis_SendSpeed(GIMBAL_AXIS_Y, 0, 0U);

    return ((status_x == HAL_OK) && (status_y == HAL_OK)) ? 1U : 0U;
}

static void UpdateEstimatedYAngle(uint32_t now_tick)
{
    uint32_t elapsed_ms = now_tick - y_angle_update_tick;

    y_angle_update_tick = now_tick;
    if (elapsed_ms > 100U) {
        elapsed_ms = 100U;
    }

    /* 1 RPM = 6度/秒。只使用确认发送成功的上一条速度估算。 */
    y_estimated_angle_deg += (float)last_speed_y * 6.0f * ((float)elapsed_ms / 1000.0f);

    if (y_estimated_angle_deg > Y_SOFT_LIMIT_DEG) {
        y_estimated_angle_deg = Y_SOFT_LIMIT_DEG;
    } else if (y_estimated_angle_deg < -Y_SOFT_LIMIT_DEG) {
        y_estimated_angle_deg = -Y_SOFT_LIMIT_DEG;
    }
}

static float Direction_GetRawError(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        return (float)target_x - CAM_CENTER_X;
    }

    return (float)target_y - CAM_CENTER_Y;
}

static int16_t Direction_GetRawCoord(uint8_t axis)
{
    return (axis == GIMBAL_AXIS_X) ? target_x : target_y;
}

static void Direction_SetVerified(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        direction_verified_x = 1U;
        direction_inconclusive_x = 0U;
        g_gimbal_debug.x_direction_verified = 1U;
        PID_Reset(&PID_X);
    } else {
        direction_verified_y = 1U;
        direction_inconclusive_y = 0U;
        g_gimbal_debug.y_direction_verified = 1U;
        PID_Reset(&PID_Y);
    }
}

static uint8_t Direction_FlipWasUsed(uint8_t axis)
{
    return (axis == GIMBAL_AXIS_X) ? direction_flip_used_x : direction_flip_used_y;
}

static void Direction_InvertAxis(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        direction_inverted_x ^= 1U;
        direction_flip_used_x = 1U;
        direction_inconclusive_x = 0U;
        g_gimbal_debug.x_direction_inverted = direction_inverted_x;
    } else {
        direction_inverted_y ^= 1U;
        direction_flip_used_y = 1U;
        direction_inconclusive_y = 0U;
        g_gimbal_debug.y_direction_inverted = direction_inverted_y;
    }
}

static uint8_t Direction_IncrementInconclusive(uint8_t axis)
{
    if (axis == GIMBAL_AXIS_X) {
        if (direction_inconclusive_x < 0xFFU) {
            direction_inconclusive_x++;
        }
        return direction_inconclusive_x;
    }

    if (direction_inconclusive_y < 0xFFU) {
        direction_inconclusive_y++;
    }
    return direction_inconclusive_y;
}

static void Direction_SetState(uint8_t state, uint8_t axis, uint32_t now_tick)
{
    direction_test_axis = axis;
    direction_state_tick = now_tick;
    g_gimbal_debug.direction_state = state;
    g_gimbal_debug.test_axis = axis;
}

static void Direction_BeginPrepare(uint8_t axis, uint32_t now_tick)
{
    direction_prepare_coord = Direction_GetRawCoord(axis);
    direction_prepare_sample_count = valid_target_sample_count;
    Direction_SetState(GIMBAL_DIR_PREPARING, axis, now_tick);
}

static void Direction_EnterFault(uint8_t axis, uint8_t reason, uint32_t now_tick)
{
    g_gimbal_debug.fault_axis = axis;
    g_gimbal_debug.fault_reason = reason;
    Direction_SetState(GIMBAL_DIR_FAULT, axis, now_tick);
}

static uint8_t Direction_SelectAxis(void)
{
    float abs_error_x = AbsFloat(Direction_GetRawError(GIMBAL_AXIS_X));
    float abs_error_y = AbsFloat(Direction_GetRawError(GIMBAL_AXIS_Y));

    if ((direction_verified_x == 0U) && (abs_error_x >= DIRECTION_TEST_MIN_ERROR_PX)) {
        return GIMBAL_AXIS_X;
    }
    if ((direction_verified_y == 0U) && (abs_error_y >= DIRECTION_TEST_MIN_ERROR_PX)) {
        return GIMBAL_AXIS_Y;
    }

    return GIMBAL_AXIS_NONE;
}

static void Direction_StartTest(uint32_t now_tick)
{
    float raw_error = Direction_GetRawError(direction_test_axis);
    int32_t test_rpm;

    if (AbsFloat(raw_error) < DIRECTION_TEST_MIN_ERROR_PX) {
        Direction_SetState(GIMBAL_DIR_WAITING, GIMBAL_AXIS_NONE, now_tick);
        return;
    }

    test_rpm = (raw_error > 0.0f) ? DIRECTION_TEST_RPM : -DIRECTION_TEST_RPM;
    if (Axis_SendSpeed(direction_test_axis, test_rpm, 0U) != HAL_OK) {
        return;
    }

    direction_test_start_abs_error = AbsFloat(raw_error);
    direction_test_start_sample_count = valid_target_sample_count;
    g_gimbal_debug.test_start_coord = Direction_GetRawCoord(direction_test_axis);
    g_gimbal_debug.test_start_abs_error = direction_test_start_abs_error;
    g_gimbal_debug.test_end_coord = g_gimbal_debug.test_start_coord;
    g_gimbal_debug.test_end_abs_error = direction_test_start_abs_error;
    g_gimbal_debug.direction_test_count++;
    Direction_SetState(GIMBAL_DIR_RUNNING, direction_test_axis, HAL_GetTick());
}

static void Direction_EvaluateTest(uint32_t now_tick)
{
    uint8_t axis = direction_test_axis;
    float end_abs_error = AbsFloat(Direction_GetRawError(axis));

    g_gimbal_debug.test_end_coord = Direction_GetRawCoord(axis);
    g_gimbal_debug.test_end_abs_error = end_abs_error;

    if (end_abs_error <= (direction_test_start_abs_error - DIRECTION_TEST_MIN_CHANGE_PX)) {
        Direction_SetVerified(axis);
        if ((direction_verified_x != 0U) && (direction_verified_y != 0U)) {
            Direction_SetState(GIMBAL_DIR_READY, GIMBAL_AXIS_NONE, now_tick);
        } else {
            Direction_SetState(GIMBAL_DIR_WAITING, GIMBAL_AXIS_NONE, now_tick);
        }
        return;
    }

    if (end_abs_error >= (direction_test_start_abs_error + DIRECTION_TEST_MIN_CHANGE_PX)) {
        /* 误差明显增大：已经先发送0RPM停止，现在只反转本轴映射并低速复验一次。 */
        if (Direction_FlipWasUsed(axis) == 0U) {
            Direction_InvertAxis(axis);
            Direction_BeginPrepare(axis, now_tick);
        } else {
            Direction_EnterFault(axis, GIMBAL_FAULT_BOTH_DIRECTIONS, now_tick);
        }
        return;
    }

    /* 坐标变化太小时不猜方向，保持停止并有限次重试。 */
    if (Direction_IncrementInconclusive(axis) >= DIRECTION_TEST_MAX_INCONCLUSIVE) {
        Direction_EnterFault(axis, GIMBAL_FAULT_NO_MOVEMENT, now_tick);
    } else {
        Direction_BeginPrepare(axis, now_tick);
    }
}

/**
 * @brief 执行单轴方向自检状态机。
 * @retval 1=本周期由自检占用，不运行正常PID；0=可运行已验证轴的正常控制。
 */
static uint8_t Direction_VerifyLoop(uint32_t now_tick)
{
    uint8_t state = g_gimbal_debug.direction_state;

    if (state == GIMBAL_DIR_READY) {
        return 0U;
    }
    if (state == GIMBAL_DIR_FAULT) {
        return 1U;
    }

    if (state == GIMBAL_DIR_WAITING) {
        uint8_t axis;

        if ((direction_verified_x != 0U) && (direction_verified_y != 0U)) {
            Direction_SetState(GIMBAL_DIR_READY, GIMBAL_AXIS_NONE, now_tick);
            return 0U;
        }

        axis = Direction_SelectAxis();
        if (axis == GIMBAL_AXIS_NONE) {
            return 0U;
        }

        /* 每次测试前明确给两轴发0RPM，保证只有待测轴会运动。 */
        if (ZDT_ForceStopBoth() != 0U) {
            Direction_BeginPrepare(axis, HAL_GetTick());
        }
        return 1U;
    }

    if (state == GIMBAL_DIR_PREPARING) {
        uint32_t new_samples = valid_target_sample_count - direction_prepare_sample_count;
        int32_t coord_change = (int32_t)Direction_GetRawCoord(direction_test_axis) -
                               (int32_t)direction_prepare_coord;

        if (AbsFloat(Direction_GetRawError(direction_test_axis)) < DIRECTION_TEST_MIN_ERROR_PX) {
            Direction_SetState(GIMBAL_DIR_WAITING, GIMBAL_AXIS_NONE, now_tick);
            return 0U;
        }

        if (((now_tick - direction_state_tick) >= DIRECTION_TEST_PREPARE_MS) &&
            (new_samples >= DIRECTION_TEST_PREPARE_SAMPLES)) {
            if (AbsInt32(coord_change) > DIRECTION_TEST_STABLE_CHANGE_PX) {
                Direction_BeginPrepare(direction_test_axis, now_tick);
            } else {
                Direction_StartTest(now_tick);
            }
        }
        return 1U;
    }

    if (state == GIMBAL_DIR_RUNNING) {
        if ((now_tick - direction_state_tick) >= DIRECTION_TEST_RUN_MS) {
            if (Axis_SendSpeed(direction_test_axis, 0, 0U) == HAL_OK) {
                Direction_SetState(GIMBAL_DIR_SETTLING, direction_test_axis, HAL_GetTick());
            }
        }
        return 1U;
    }

    if (state == GIMBAL_DIR_SETTLING) {
        uint32_t new_samples = valid_target_sample_count - direction_test_start_sample_count;

        if (((now_tick - direction_state_tick) >= DIRECTION_TEST_SETTLE_MS) &&
            (new_samples >= DIRECTION_TEST_MIN_SAMPLES)) {
            Direction_EvaluateTest(now_tick);
        }
        return 1U;
    }

    Direction_SetState(GIMBAL_DIR_WAITING, GIMBAL_AXIS_NONE, now_tick);
    return 0U;
}

static void Direction_CancelActiveTest(uint32_t now_tick)
{
    uint8_t state = g_gimbal_debug.direction_state;

    if ((state == GIMBAL_DIR_PREPARING) ||
        (state == GIMBAL_DIR_RUNNING) ||
        (state == GIMBAL_DIR_SETTLING)) {
        Direction_SetState(GIMBAL_DIR_WAITING, GIMBAL_AXIS_NONE, now_tick);
    }
}

/**
 * @brief 云台控制初始化。
 */
void Gimbal_Init(void)
{
    uint8_t motor_init_ok = 1U;

    filtered_x = CAM_CENTER_X;
    filtered_y = CAM_CENTER_Y;
    last_update_time = HAL_GetTick();
    valid_target_sample_count = 0U;
    last_speed_x = 0;
    last_speed_y = 0;
    y_estimated_angle_deg = 0.0f;
    y_angle_update_tick = HAL_GetTick();
    tracking_was_active = 0U;
    target_found = 0U;

    direction_verified_x = 0U;
    direction_verified_y = 0U;
    direction_inverted_x = 0U;
    direction_inverted_y = 0U;
    direction_flip_used_x = 0U;
    direction_flip_used_y = 0U;
    direction_inconclusive_x = 0U;
    direction_inconclusive_y = 0U;
    direction_prepare_sample_count = 0U;
    direction_prepare_coord = 0;
    direction_test_start_sample_count = 0U;
    direction_test_start_abs_error = 0.0f;

    g_gimbal_debug.direction_state = GIMBAL_DIR_WAITING;
    g_gimbal_debug.test_axis = GIMBAL_AXIS_NONE;
    g_gimbal_debug.x_direction_verified = 0U;
    g_gimbal_debug.y_direction_verified = 0U;
    g_gimbal_debug.x_direction_inverted = 0U;
    g_gimbal_debug.y_direction_inverted = 0U;
    g_gimbal_debug.fault_axis = GIMBAL_AXIS_NONE;
    g_gimbal_debug.fault_reason = GIMBAL_FAULT_NONE;
    g_gimbal_debug.last_motor_tx_status = (uint8_t)HAL_OK;
    g_gimbal_debug.last_motor_tx_status_x = (uint8_t)HAL_OK;
    g_gimbal_debug.last_motor_tx_status_y = (uint8_t)HAL_OK;
    g_gimbal_debug.valid_target_sample_count = 0U;
    g_gimbal_debug.direction_test_count = 0U;
    g_gimbal_debug.test_start_coord = 0;
    g_gimbal_debug.test_end_coord = 0;
    g_gimbal_debug.test_start_abs_error = 0.0f;
    g_gimbal_debug.test_end_abs_error = 0.0f;
    g_gimbal_debug.commanded_rpm_x = 0;
    g_gimbal_debug.commanded_rpm_y = 0;

    PID_Init(&PID_X, PID_KP_RPM, PID_KI_RPM, PID_KD_RPM, PID_MAX_OUTPUT_RPM, PID_MAX_INTEGRAL);
    PID_Init(&PID_Y, PID_KP_RPM, PID_KI_RPM, PID_KD_RPM, PID_MAX_OUTPUT_RPM, PID_MAX_INTEGRAL);

    HAL_Delay(500U);
    MotorBus_StartReplyDrain();

    /* 堵转解除只在系统初始化执行这一次。运行中不会自动重复解锁。 */
    if (MotorBus_WaitTxReady() == HAL_OK) {
        Emm_V5_Reset_Clog_Pro(ZDT_MOTOR_X_ADDR);
        if (MotorBus_RecordLastResult() == 0U) {
            motor_init_ok = 0U;
        }
    } else {
        motor_init_ok = 0U;
    }

    if (MotorBus_WaitTxReady() == HAL_OK) {
        Emm_V5_Reset_Clog_Pro(ZDT_MOTOR_Y_ADDR);
        if (MotorBus_RecordLastResult() == 0U) {
            motor_init_ok = 0U;
        }
    } else {
        motor_init_ok = 0U;
    }

    if (MotorBus_WaitTxReady() == HAL_OK) {
        Emm_V5_En_Control(ZDT_MOTOR_X_ADDR, true, false);
        if (MotorBus_RecordLastResult() == 0U) {
            motor_init_ok = 0U;
        }
    } else {
        motor_init_ok = 0U;
    }

    if (MotorBus_WaitTxReady() == HAL_OK) {
        Emm_V5_En_Control(ZDT_MOTOR_Y_ADDR, true, false);
        if (MotorBus_RecordLastResult() == 0U) {
            motor_init_ok = 0U;
        }
    } else {
        motor_init_ok = 0U;
    }

    /* 清除驱动器可能保留的旧速度指令。 */
    if (ZDT_ForceStopBoth() == 0U) {
        motor_init_ok = 0U;
    }

#if GIMBAL_AUTO_DIRECTION_VERIFY == 0U
    direction_verified_x = 1U;
    direction_verified_y = 1U;
    g_gimbal_debug.x_direction_verified = 1U;
    g_gimbal_debug.y_direction_verified = 1U;
    Direction_SetState(GIMBAL_DIR_READY, GIMBAL_AXIS_NONE, HAL_GetTick());
#else
    Direction_SetState(GIMBAL_DIR_WAITING, GIMBAL_AXIS_NONE, HAL_GetTick());
#endif

    if (motor_init_ok == 0U) {
        Direction_EnterFault(GIMBAL_AXIS_NONE, GIMBAL_FAULT_MOTOR_INIT, HAL_GetTick());
    }

    Led_Set(LED_OFF_LEVEL);
}

/**
 * @brief 更新视觉目标。
 */
void Gimbal_UpdateTarget(int16_t x, int16_t y, uint8_t is_found)
{
    uint8_t was_found = target_found;

    target_x = x;
    target_y = y;
    target_found = is_found;

    if (is_found != 0U) {
        /* 目标重新出现时直接用新坐标重置滤波，避免沿用丢失前的位置而短时反向。 */
        if (was_found == 0U) {
            filtered_x = (float)x;
            filtered_y = (float)y;
        } else {
            filtered_x = EMA_ALPHA * (float)x + (1.0f - EMA_ALPHA) * filtered_x;
            filtered_y = EMA_ALPHA * (float)y + (1.0f - EMA_ALPHA) * filtered_y;
        }

        valid_target_sample_count++;
        g_gimbal_debug.valid_target_sample_count = valid_target_sample_count;
    }

    last_update_time = HAL_GetTick();
}

/**
 * @brief 云台20ms控制循环。
 */
void Gimbal_Loop(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint8_t is_timeout = ((now_tick - last_update_time) > 250U) ? 1U : 0U;

    UpdateEstimatedYAngle(now_tick);

    if (is_timeout != 0U) {
        target_found = 0U;
    }

    /* 方向复验失败后锁停，不在追踪过程中解除堵转保护或反复换向。 */
    if (g_gimbal_debug.direction_state == GIMBAL_DIR_FAULT) {
        if ((last_speed_x != 0) || (last_speed_y != 0)) {
            (void)ZDT_ForceStopBoth();
        }
        PID_Reset(&PID_X);
        PID_Reset(&PID_Y);
        tracking_was_active = 0U;
        Led_Set(LED_OFF_LEVEL);
        return;
    }

    if (target_found != 0U) {
        float pixel_error_x;
        float pixel_error_y;
        int32_t speed_x = 0;
        int32_t speed_y = 0;

        if (Direction_VerifyLoop(now_tick) != 0U) {
            PID_Reset(&PID_X);
            PID_Reset(&PID_Y);
            tracking_was_active = 1U;
            Led_TrackingBlink();
            return;
        }

        /* 保留已观察到的符号含义：坐标大于中心时 PID.output > 0。
         * 物理电机方向由上面的低速坐标反馈自检决定。
         */
        pixel_error_x = filtered_x - CAM_CENTER_X;
        pixel_error_y = filtered_y - CAM_CENTER_Y;

        if (direction_verified_x == 0U) {
            PID_Reset(&PID_X);
        } else if (AbsFloat(pixel_error_x) < DEADZONE_X) {
            PID_Reset(&PID_X);
        } else {
            speed_x = PID_OutputToRpm(PID_Calculate(&PID_X, pixel_error_x, 0.0f));
        }

        if (direction_verified_y == 0U) {
            PID_Reset(&PID_Y);
        } else if (AbsFloat(pixel_error_y) < DEADZONE_Y) {
            PID_Reset(&PID_Y);
        } else {
            speed_y = PID_OutputToRpm(PID_Calculate(&PID_Y, pixel_error_y, 0.0f));
        }

        /* Y 轴到软限位后禁止继续向外转，但允许立即反向退回。 */
        if (((y_estimated_angle_deg >= Y_SOFT_LIMIT_DEG) && (speed_y > 0)) ||
            ((y_estimated_angle_deg <= -Y_SOFT_LIMIT_DEG) && (speed_y < 0))) {
            speed_y = 0;
            PID_Reset(&PID_Y);
        }

        if (speed_x != last_speed_x) {
            (void)Axis_SendSpeed(GIMBAL_AXIS_X, speed_x, ZDT_TRACK_ACC);
        }
        if (speed_y != last_speed_y) {
            (void)Axis_SendSpeed(GIMBAL_AXIS_Y, speed_y, ZDT_TRACK_ACC);
        }

        tracking_was_active = 1U;
        Led_TrackingBlink();
    } else {
        Direction_CancelActiveTest(now_tick);

        /* 丢失目标后持续重试停止，直到两条0RPM命令都确认发送成功。 */
        if ((tracking_was_active != 0U) || (last_speed_x != 0) || (last_speed_y != 0)) {
            (void)ZDT_ForceStopBoth();
        }
        tracking_was_active = 0U;
        PID_Reset(&PID_X);
        PID_Reset(&PID_Y);
        Led_Set(LED_OFF_LEVEL);
    }
}
