#include "stm32_pca9685.h"
#include "iic.h"
#include "delay.h"

void PCA9685_Init(float hz,u8 angle)
{
	u32 off = 0;
	IIC_Init();
	PCA9685_Write(PCA_Model,0x00);
	PCA9685_setFreq(hz);
	off = (u32)(145+angle*2.4);
	PCA9685_setPWM(0,0,off);
	PCA9685_setPWM(1,0,off);
	PCA9685_setPWM(2,0,off);
	PCA9685_setPWM(3,0,off);
	PCA9685_setPWM(4,0,off);
	PCA9685_setPWM(5,0,off);
	PCA9685_setPWM(6,0,off);
	PCA9685_setPWM(7,0,off);
	PCA9685_setPWM(8,0,off);
	PCA9685_setPWM(9,0,off);
	PCA9685_setPWM(10,0,off);
	PCA9685_setPWM(11,0,off);
	PCA9685_setPWM(12,0,off);
	PCA9685_setPWM(13,0,off);
	PCA9685_setPWM(14,0,off);
	PCA9685_setPWM(15,0,off);

	delay_ms(100);

}

void PCA9685_Write(u8 addr,u8 data)
{
	IIC_Start();

	IIC_Send_Byte(PCA_Addr);
	IIC_NAck();

	IIC_Send_Byte(addr);
	IIC_NAck();

	IIC_Send_Byte(data);
	IIC_NAck();

	IIC_Stop();


}

u8 PCA9685_Read(u8 addr)
{
	u8 data;

	IIC_Start();

	IIC_Send_Byte(PCA_Addr);
	IIC_NAck();

	IIC_Send_Byte(addr);
	IIC_NAck();

	IIC_Stop();

	delay_us(10);


	IIC_Start();

	IIC_Send_Byte(PCA_Addr|0x01);
	IIC_NAck();

	data = IIC_Read_Byte(0);

	IIC_Stop();

	return data;

}

void PCA9685_setPWM(u8 num,u32 on,u32 off)
{
	IIC_Start();

	IIC_Send_Byte(PCA_Addr);
	IIC_Wait_Ack();

	IIC_Send_Byte(LED0_ON_L+4*num);
	IIC_Wait_Ack();

	IIC_Send_Byte(on&0xFF);
	IIC_Wait_Ack();

	IIC_Send_Byte(on>>8);
	IIC_Wait_Ack();

	IIC_Send_Byte(off&0xFF);
	IIC_Wait_Ack();

	IIC_Send_Byte(off>>8);
	IIC_Wait_Ack();

	IIC_Stop();

}

void PCA9685_setFreq(float freq)
{
	u8 prescale,oldmode,newmode;

	double prescaleval;

	//freq *= 0.92;
	prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	prescale = (u8)(prescaleval+0.5f);
	oldmode = PCA9685_Read(PCA_Model);

	newmode = (oldmode&0x7F)|0x10;
	PCA9685_Write(PCA_Model,newmode);
	PCA9685_Write(PCA_Pre,prescale);
	PCA9685_Write(PCA_Model,oldmode);
	delay_ms(5);
	PCA9685_Write(PCA_Model,oldmode|0xa1);


}

void setAngle(u8 num,u8 angle)
{
	u32 off = 0;
	off = (u32)(158+angle*2.2);
	PCA9685_setPWM(num,0,off);
}