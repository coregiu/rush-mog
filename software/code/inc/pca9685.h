#ifndef PCA9685_H
#define PCA9685_H

#include <mcs51/8051.h>
#include <stdio.h>
#include "command_receiver.h"

#define sda P1_0 //数据输入/输出端
#define scl P1_1 //时钟输入线

#define PCA9685_adrr 0x80 //  1+A5+A4+A3+A2+A1+A0+w/r
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

//#define SERVO_ANGLE0  102 //0度对应4096的脉宽计数值
//#define SERVO_ANGLE180  476 //180度对应4096的脉宽计算值，根据不同舵机修改

void init_pca9685();
void init_iic();
void start_iic();
void stop_iic();
void ack_iic();
void reset_pca9685(void);
void write_byte(uchar byte);
uchar pca9685_read(uchar address);
uchar read_byte();
void pca9685_write(uchar address, uchar date);
void set_pwm_freq(float freq);
void servo_control(uchar num, uchar angle);

#endif