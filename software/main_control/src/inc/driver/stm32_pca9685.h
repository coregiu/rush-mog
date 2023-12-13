#include "stm32f10x.h"

#define PCA_Addr 0x80
#define PCA_Model 0x00
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define PCA_Pre 0xFE

void PCA9685_Init(float hz, u8 angle);

void PCA9685_Write(u8 addr, u8 data);

u8 PCA9685_Read(u8 addr);

void PCA9685_setPWM(u8 num, u32 on, u32 off);

void PCA9685_setFreq(float freq);

void setAngle(u8 num, u8 angle);