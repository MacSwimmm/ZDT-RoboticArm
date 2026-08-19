/*
 * Key.c
 * --------------------------------------------------------------------------
 * 六个独立按键的消抖、短按和长按识别。
 *
 * 本模块不解释 K1~K6 的业务含义，只输出标准化按键事件。页面导航、
 * 使能、停止和清零等动作由上层 ArmControl 决定，因此按键驱动可以在
 * 其他项目中复用而无需修改底层扫描代码。
 */

#include "Key.h"

#define KEY_EVENT_QUEUE_SIZE       64U
#define KEY_DEBOUNCE_MS            20U
#define KEY_LONG_PRESS_MS          800U

static volatile KeyEvent_t key_event_queue[KEY_EVENT_QUEUE_SIZE];
static volatile uint8_t key_queue_head;
static volatile uint8_t key_queue_tail;

static uint8_t key_raw[KEY_COUNT];
static uint8_t key_stable[KEY_COUNT];
static uint8_t key_debounce_count[KEY_COUNT];
static uint16_t key_hold_ms[KEY_COUNT];
static uint8_t key_long_sent[KEY_COUNT];

static GPIO_PinState Key_ReadGPIO(uint8_t index)
{
    switch (index) {
    case 0U: return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    case 1U: return HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
    case 2U: return HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
    case 3U: return HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin);
    case 4U: return HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin);
    default: return HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin);
    }
}

static void Key_PushEvent(KeyEventType_t type, uint8_t index)
{
    uint8_t next = (uint8_t)((key_queue_head + 1U) % KEY_EVENT_QUEUE_SIZE);

    /* 队列满时丢弃最新事件，避免覆盖主循环尚未处理的操作。 */
    if (next == key_queue_tail) {
        return;
    }

    key_event_queue[key_queue_head].type = type;
    key_event_queue[key_queue_head].index = index;
    key_queue_head = next;
}

void Key_Init(void)
{
    uint8_t index;

    key_queue_head = 0U;
    key_queue_tail = 0U;
    for (index = 0U; index < KEY_COUNT; ++index) {
        key_raw[index] = (Key_ReadGPIO(index) == GPIO_PIN_RESET) ? 1U : 0U;
        key_stable[index] = key_raw[index];
        key_debounce_count[index] = 0U;
        key_hold_ms[index] = 0U;
        key_long_sent[index] = 0U;
    }
}

void Key_Scan1ms(void)
{
    uint8_t index;

    for (index = 0U; index < KEY_COUNT; ++index) {
        uint8_t raw = (Key_ReadGPIO(index) == GPIO_PIN_RESET) ? 1U : 0U;

        if (raw != key_raw[index]) {
            /* 原始电平变化，重新开始 20 ms 稳定计时。 */
            key_raw[index] = raw;
            key_debounce_count[index] = 0U;
        } else if (key_debounce_count[index] < KEY_DEBOUNCE_MS) {
            ++key_debounce_count[index];
        } else if (raw != key_stable[index]) {
            /* 只有稳定超过消抖时间后，才接受新的按下/释放状态。 */
            key_stable[index] = raw;
            if (raw != 0U) {
                key_hold_ms[index] = 0U;
                key_long_sent[index] = 0U;
            } else if (key_long_sent[index] == 0U) {
                Key_PushEvent(KEY_EVENT_SHORT, index);
            }
        } else if ((key_stable[index] != 0U) &&
                   (key_long_sent[index] == 0U)) {
            if (key_hold_ms[index] < KEY_LONG_PRESS_MS) {
                ++key_hold_ms[index];
            }
            if (key_hold_ms[index] >= KEY_LONG_PRESS_MS) {
                key_long_sent[index] = 1U;
                Key_PushEvent(KEY_EVENT_LONG, index);
            }
        }
    }
}

uint8_t Key_PollEvent(KeyEvent_t *event)
{
    /* TIM6 中断只推进 head，主循环只推进 tail，职责边界清晰。 */
    if ((event == 0) || (key_queue_tail == key_queue_head)) {
        return 0U;
    }

    *event = key_event_queue[key_queue_tail];
    key_queue_tail = (uint8_t)((key_queue_tail + 1U) % KEY_EVENT_QUEUE_SIZE);
    return 1U;
}
