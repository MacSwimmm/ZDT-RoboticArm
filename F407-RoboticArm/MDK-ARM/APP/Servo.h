#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);
uint8_t Servo_GetAngle(void);

#endif
