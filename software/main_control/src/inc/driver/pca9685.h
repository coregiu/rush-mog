/**
  ******************************************************************************
  *
  * Define of pca9685 driver.
  * author: from https://gitee.com/qi-zezhong/pca9685-stm32
  *
  *
  ******************************************************************************
**/


#ifndef __STM32PCA9685_H
#define __STM32PCA9685_H

#include "iic.h"

#define pca_adrr 0x80

#define pca_mode1 0x0
#define pca_pre 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define jdMIN 115 // minimum
#define jdMAX 590 // maximum
#define jd000 130 //0度对应4096的脉宽计数值
#define jd180 520 //180度对应4096的脉宽计算值



void pca_init(float hz);
void servo_init(u8 num, u8 angle);

void pca_write(u8 adrr, u8 data);
u8 pca_read(u8 adrr);
void pca_setfreq(float freq);
void pca_setpwm(u8 num, u32 on, u32 off);
void pca_mg9xx(u8 num, u8 start_angle, u8 end_angle, u8 mode, u8 speed);

#endif
