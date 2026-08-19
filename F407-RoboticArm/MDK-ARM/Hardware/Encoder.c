/*
 * Encoder.c
 * --------------------------------------------------------------------------
 * 四路 EC11 正交编码器驱动。
 *
 * 本模块只处理 GPIO 采样、正交状态机、编码器按压消抖和事件排队，
 * 不知道电机地址、OLED 页面或参数调节逻辑，因此可以独立移植到其他
 * STM32 工程。调用者只需要在固定周期调用 Encoder_Scan1ms()，再在主循环
 * 中使用 Encoder_PollEvent() 取出事件即可。
 */

#include "Encoder.h"

#define ENCODER_EVENT_QUEUE_SIZE       64U
#define ENCODER_BUTTON_DEBOUNCE_MS     20U

static volatile EncoderEvent_t encoder_event_queue[ENCODER_EVENT_QUEUE_SIZE];
static volatile uint8_t encoder_queue_head;
static volatile uint8_t encoder_queue_tail;

static uint8_t encoder_last_state[ENCODER_COUNT];
static int8_t encoder_accumulator[ENCODER_COUNT];
static uint8_t encoder_press_last[ENCODER_COUNT];
static uint8_t encoder_press_raw[ENCODER_COUNT];
static uint8_t encoder_press_count[ENCODER_COUNT];

/*
 * 标准两相正交编码器状态转移表。
 * 索引的高两位是上一次 AB 状态，低两位是当前 AB 状态。
 * 合法的四步状态序列会累计成一个完整的旋转事件，抖动和非法跳变
 * 不会产生有效的 delta。
 */
static const int8_t encoder_quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t Encoder_ReadAB(uint8_t index)
{
    GPIO_PinState a;
    GPIO_PinState b;

    switch (index) {
    case 0U:
        a = HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin);
        b = HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin);
        break;
    case 1U:
        a = HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin);
        b = HAL_GPIO_ReadPin(ENC2_B_GPIO_Port, ENC2_B_Pin);
        break;
    case 2U:
        a = HAL_GPIO_ReadPin(ENC3_A_GPIO_Port, ENC3_A_Pin);
        b = HAL_GPIO_ReadPin(ENC3_B_GPIO_Port, ENC3_B_Pin);
        break;
    default:
        a = HAL_GPIO_ReadPin(ENC4_A_GPIO_Port, ENC4_A_Pin);
        b = HAL_GPIO_ReadPin(ENC4_B_GPIO_Port, ENC4_B_Pin);
        break;
    }

    return (uint8_t)(((a == GPIO_PIN_SET) ? 2U : 0U) |
                     ((b == GPIO_PIN_SET) ? 1U : 0U));
}

static GPIO_PinState Encoder_ReadPush(uint8_t index)
{
    switch (index) {
    case 0U: return HAL_GPIO_ReadPin(ENC1_E_GPIO_Port, ENC1_E_Pin);
    case 1U: return HAL_GPIO_ReadPin(ENC2_E_GPIO_Port, ENC2_E_Pin);
    case 2U: return HAL_GPIO_ReadPin(ENC3_E_GPIO_Port, ENC3_E_Pin);
    default: return HAL_GPIO_ReadPin(ENC4_E_GPIO_Port, ENC4_E_Pin);
    }
}

static void Encoder_PushEvent(EncoderEventType_t type, uint8_t index, int8_t delta)
{
    uint8_t next = (uint8_t)((encoder_queue_head + 1U) % ENCODER_EVENT_QUEUE_SIZE);

    /* 队列满时丢弃最新事件，避免覆盖尚未处理的旧事件。 */
    if (next == encoder_queue_tail) {
        return;
    }

    encoder_event_queue[encoder_queue_head].type = type;
    encoder_event_queue[encoder_queue_head].index = index;
    encoder_event_queue[encoder_queue_head].delta = delta;
    encoder_queue_head = next;
}

void Encoder_Init(void)
{
    uint8_t index;

    encoder_queue_head = 0U;
    encoder_queue_tail = 0U;
    for (index = 0U; index < ENCODER_COUNT; ++index) {
        encoder_last_state[index] = Encoder_ReadAB(index);
        encoder_accumulator[index] = 0;

        /* 编码器按键为低电平有效，上电时记录当前稳定电平。 */
        encoder_press_last[index] =
            (Encoder_ReadPush(index) == GPIO_PIN_RESET) ? 1U : 0U;
        encoder_press_raw[index] = encoder_press_last[index];
        encoder_press_count[index] = 0U;
    }
}

void Encoder_Scan1ms(void)
{
    uint8_t index;

    for (index = 0U; index < ENCODER_COUNT; ++index) {
        uint8_t current = Encoder_ReadAB(index);
        uint8_t transition = (uint8_t)((encoder_last_state[index] << 2) | current);
        int8_t movement = encoder_quadrature_table[transition & 0x0FU];

        encoder_last_state[index] = current;
        encoder_accumulator[index] += movement;
        if (encoder_accumulator[index] >= 4) {
            Encoder_PushEvent(ENCODER_EVENT_ROTATE, index, 1);
            encoder_accumulator[index] = 0;
        } else if (encoder_accumulator[index] <= -4) {
            Encoder_PushEvent(ENCODER_EVENT_ROTATE, index, -1);
            encoder_accumulator[index] = 0;
        }

        /* 编码器按压脚只产生按下事件，暂不参与菜单逻辑。 */
        {
            uint8_t raw = (Encoder_ReadPush(index) == GPIO_PIN_RESET) ? 1U : 0U;
            if (raw != encoder_press_raw[index]) {
                encoder_press_raw[index] = raw;
                encoder_press_count[index] = 0U;
            } else if (encoder_press_count[index] < ENCODER_BUTTON_DEBOUNCE_MS) {
                ++encoder_press_count[index];
            } else if (raw != encoder_press_last[index]) {
                encoder_press_last[index] = raw;
                if (raw != 0U) {
                    Encoder_PushEvent(ENCODER_EVENT_PRESS, index, 0);
                }
            }
        }
    }
}

uint8_t Encoder_PollEvent(EncoderEvent_t *event)
{
    /*
     * 扫描函数可能在 TIM6 中断中推进 head，主循环只推进 tail。
     * uint8_t 的单次读写在 Cortex-M4 上是原子的，因此无需在此关中断。
     */
    if ((event == 0) || (encoder_queue_tail == encoder_queue_head)) {
        return 0U;
    }

    *event = encoder_event_queue[encoder_queue_tail];
    encoder_queue_tail =
        (uint8_t)((encoder_queue_tail + 1U) % ENCODER_EVENT_QUEUE_SIZE);
    return 1U;
}
