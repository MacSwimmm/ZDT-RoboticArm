#ifndef __ARM_ENCODER_H
#define __ARM_ENCODER_H

#include "main.h"

/* 四个 EC11 编码器对应四个电机，编号范围为 0~3。 */
#define ENCODER_COUNT       4U

typedef enum {
    ENCODER_EVENT_ROTATE = 0,       /* A/B 相旋转事件，delta 为 +1 或 -1。 */
    ENCODER_EVENT_PRESS             /* 编码器按压事件，当前只做预留。 */
} EncoderEventType_t;

typedef struct {
    EncoderEventType_t type;
    uint8_t index;                  /* 编码器编号：0~3。 */
    int8_t delta;                   /* 旋转方向：+1 或 -1。 */
} EncoderEvent_t;

/* 记录四路 A/B 相及按压脚的当前电平，并清空编码器事件队列。 */
void Encoder_Init(void);

/* 每 1 ms 调用一次，仅采样 GPIO、运行状态机并将事件放入队列。 */
void Encoder_Scan1ms(void);

/* 从编码器队列取出一个事件；有事件返回 1，队列为空或参数非法返回 0。 */
uint8_t Encoder_PollEvent(EncoderEvent_t *event);

#endif
