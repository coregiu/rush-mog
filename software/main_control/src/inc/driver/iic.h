#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"

//IO·½ÏòÉèÖÃ
#define SDA_IN()  {GPIOA->CRH&=0XFFFF0FFF;GPIOA->CRH|=8<<12;}
#define SDA_OUT() {GPIOA->CRH&=0XFFFF0FFF;GPIOA->CRH|=3<<12;}

//IO²Ù×÷º¯Êý
#define IIC_SCL    PAout(12) //SCL
#define IIC_SDA    PAout(11) //SDA
#define READ_SDA   PAin(11)  //ÊäÈëSDA

//IICËùÓÐ²Ù×÷º¯Êý
void IIC_Init(void);                //³õÊ¼»¯IICµÄIO¿Ú
void IIC_Start(void);				//·¢ËÍIIC¿ªÊ¼ÐÅºÅ
void IIC_Stop(void);	  			//·¢ËÍIICÍ£Ö¹ÐÅºÅ
void IIC_Send_Byte(u8 txd);			//IIC·¢ËÍÒ»¸ö×Ö½Ú
u8 IIC_Read_Byte(unsigned char ack);//IIC¶ÁÈ¡Ò»¸ö×Ö½Ú
u8 IIC_Wait_Ack(void); 				//IICµÈ´ýACKÐÅºÅ
void IIC_Ack(void);					//IIC·¢ËÍACKÐÅºÅ
void IIC_NAck(void);				//IIC²»·¢ËÍACKÐÅºÅ

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
#endif
