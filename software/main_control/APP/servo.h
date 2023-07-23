#ifndef __SERVO_H
#define __SERVO_H	 

#include "stm32f4xx.h"

void TIM4_Int_Init(u16 reload ,u16 prescaler);
void Set_Servo_Angle(uint8_t angle);

#endif
