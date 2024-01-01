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

void pca_setpwm(u8 num, u32 on, u32 off)
{
    pca_write(LED0_ON_L + 4 * num, on);
    pca_write(LED0_ON_H + 4 * num, on >> 8);
    pca_write(LED0_OFF_L + 4 * num, off);
    pca_write(LED0_OFF_H + 4 * num, off >> 8);
}
/*num:舵机PWM输出引脚0~15，on:PWM上升计数值0~4096,off:PWM下降计数值0~4096
一个PWM周期分成4096份，由0开始+1计数，计到on时跳变为高电平，继续计数到off时
跳变为低电平，直到计满4096重新开始。所以当on不等于0时可作延时,当on等于0时，
off/4096的值就是PWM的占空比。*/

/*
	函数作用：初始化舵机驱动板
	参数：1.PWM频率
		  2.初始化舵机角度
*/
void pca_mg9xx_init(float hz, u8 angle)
{
    u32 off = 0;
    IIC_Init();
    pca_write(pca_mode1, 0x0);
    pca_setfreq(hz); //设置PWM频率
    off = (u32)(145 + angle * 2.4);
    pca_setpwm(0, 0, off);
    pca_setpwm(1, 0, off);
    pca_setpwm(2, 0, off);
    pca_setpwm(3, 0, off);
    pca_setpwm(4, 0, off);
    pca_setpwm(5, 0, off);
    pca_setpwm(6, 0, off);
    pca_setpwm(7, 0, off);
    pca_setpwm(8, 0, off);
    pca_setpwm(9, 0, off);
    pca_setpwm(10, 0, off);
    pca_setpwm(11, 0, off);
    pca_setpwm(12, 0, off);
    pca_setpwm(13, 0, off);
    pca_setpwm(14, 0, off);
    pca_setpwm(15, 0, off);
    delay_ms(500);
}

/*
	函数作用：初始化舵机驱动板
	参数： 1.舵机编号
          2.PWM频率
		  3.初始化舵机角度
*/
void pca_servo_init(u8 num, float hz, u8 angle)
{
    u32 off = 0;
    IIC_Init();
    pca_write(pca_mode1, 0x0);
    pca_setfreq(hz); //设置PWM频率
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

//----------------
//初始化IIC
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7); //PB6,PB7 输出高
}

//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT(); //sda线输出
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(4);
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT(); //sda线输出
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL = 1;
    IIC_SDA = 1; //发送I2C总线结束信号
    delay_us(4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN(); //SDA设置为输入
    IIC_SDA = 1;
    delay_us(1);
    IIC_SCL = 1;
    delay_us(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL = 0; //时钟输出0
    return 0;
}
//产生ACK应答
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}
//不产生ACK应答
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL = 0; //拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        delay_us(2); //对TEA5767这三个延时都是必须的
        IIC_SCL = 1;
        delay_us(2);
        IIC_SCL = 0;
        delay_us(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); //SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck(); //发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}
