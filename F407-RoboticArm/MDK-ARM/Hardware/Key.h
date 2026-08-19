#ifndef __ARM_KEY_H
#define __ARM_KEY_H

#include "main.h"

/* 板上六个独立按键，对外编号 0~5，对应原理图 K1~K6。 */
#define KEY_COUNT       6U

typedef enum {
    KEY_EVENT_SHORT = 0,            /* 消抖后释放，且按住时间不足 800 ms。 */
    KEY_EVENT_LONG                  /* 稳定按住达到 800 ms 时立即产生一次。 */
} KeyEventType_t;

typedef struct {
    KeyEventType_t type;
    uint8_t index;                  /* 按键编号：0~5，对应 K1~K6。 */
} KeyEvent_t;

/* 记录六个按键的当前稳定电平，并清空按键事件队列。 */
void Key_Init(void);

/* 每 1 ms 调用一次，仅负责消抖、长短按识别和事件入队。 */
void Key_Scan1ms(void);

/* 从按键队列取出一个事件；有事件返回 1，队列为空或参数非法返回 0。 */
uint8_t Key_PollEvent(KeyEvent_t *event);

#endif
