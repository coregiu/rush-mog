/**
  ******************************************************************************
  *
  * Define of pca9685 driver.
  * author: from https://gitee.com/qi-zezhong/pca9685-stm32
  *
  *
  ******************************************************************************
**/
#include "stm32_pca9685.h"

void pca_write(u8 adrr, u8 data) //向PCA写数据,adrrd地址,data数据
{
    IIC_Start();
    IIC_Send_Byte(pca_adrr);
    IIC_Wait_Ack();

    IIC_Send_Byte(adrr);
    IIC_Wait_Ack();

    IIC_Send_Byte(data);
    IIC_Wait_Ack();

    IIC_Stop();
}

u8 pca_read(u8 adrr) //从PCA读数据
{
    u8 data;
    IIC_Start();

    IIC_Send_Byte(pca_adrr);
    IIC_Wait_Ack();

    IIC_Send_Byte(adrr);
    IIC_Wait_Ack();

    IIC_Start();

    IIC_Send_Byte(pca_adrr | 0x01);
    IIC_Wait_Ack();

    data = IIC_Read_Byte(0);
    IIC_Stop();

    return data;
}

void pca_setfreq(float freq) //设置PWM频率
{
    u8 prescale, oldmode, newmode;
    double prescaleval;
    freq *= 0.92;
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    prescale = (u8)(prescaleval + 0.5f);

    oldmode = pca_read(pca_mode1);

    newmode = (oldmode & 0x7F) | 0x10; // sleep

    pca_write(pca_mode1, newmode); // go to sleep

    pca_write(pca_pre, prescale); // set the prescaler

    pca_write(pca_mode1, oldmode);
    delay_ms(2);

    pca_write(pca_mode1, oldmode | 0xa1);
}

/*num:舵机PWM输出引脚0~15，on:PWM上升计数值0~4096,off:PWM下降计数值0~4096
一个PWM周期分成4096份，由0开始+1计数，计到on时跳变为高电平，继续计数到off时
跳变为低电平，直到计满4096重新开始。所以当on不等于0时可作延时,当on等于0时，
off/4096的值就是PWM的占空比。*/
void pca_setpwm(u8 num, u32 on, u32 off)
{
    pca_write(LED0_ON_L + 4 * num, on);
    pca_write(LED0_ON_H + 4 * num, on >> 8);
    pca_write(LED0_OFF_L + 4 * num, off);
    pca_write(LED0_OFF_H + 4 * num, off >> 8);
}

/*
	函数作用：初始化舵机驱动板
	参数：1.PWM频率
*/
void pca_init(float hz)
{
    IIC_Init();
    pca_write(pca_mode1, 0x0);
    pca_setfreq(hz); //设置PWM频率
}

/*
	函数作用：初始化舵机驱动板
	参数： 1.舵机编号
		  2.初始化舵机角度
*/
void servo_init(u8 num, u8 angle)
{
    u32 off = 0;
    off = (u32)(145 + angle * 2.4);
    pca_setpwm(num, 0, off);
}

/*
	函数作用：控制舵机转动；
	参数：1.输出端口，可选0~15；
		  2.起始角度，可选0~180；
		  3.结束角度，可选0~180；
		  4.模式选择，0 表示函数内无延时，调用时需要在函数后另外加延时函数，且不可调速，第五个参数可填任意值；
					  1 表示函数内有延时，调用时不需要在函数后另外加延时函数，且不可调速，第五个参数可填任意值；
					  2 表示速度可调，第五个参数表示速度值；
		  5.速度，可填大于 0 的任意值，填 1 时速度最快，数值越大，速度越小；
	注意事项：模式 0和1 的速度比模式 2 的最大速度大；
*/
void pca_mg9xx(u8 num, u8 start_angle, u8 end_angle, u8 mode, u8 speed)
{
    u8 i;
    u32 off = 0;
    switch (mode)
    {
    case 0:
        off = (u32)(158 + end_angle * 2.2);
        pca_setpwm(num, 0, off);
        break;
    case 1:
        off = (u32)(158 + end_angle * 2.2);
        pca_setpwm(num, 0, off);
        if (end_angle > start_angle)
        {
            delay_ms((u16)((end_angle - start_angle) * 2.7));
        }
        else
        {
            delay_ms((u16)((start_angle - end_angle) * 2.7));
        }
        break;
    case 2:
        if (end_angle > start_angle)
        {
            for (i = start_angle; i <= end_angle; i++)
            {
                off = (u32)(158 + i * 2.2);
                pca_setpwm(num, 0, off);
                delay_ms(2);
                delay_us(speed * 250);
            }
        }
        else if (start_angle > end_angle)
        {
            for (i = start_angle; i >= end_angle; i--)
            {
                off = (u32)(158 + i * 2.2);
                pca_setpwm(num, 0, off);
                delay_ms(2);
                delay_us(speed * 250);
            }
        }
        break;
    }
}

