#ifndef __KEY_H
#define __KEY_H	 
 
#include "includes.h"

#define KEY_PRESS               0        //短按
#define KEY_LOOSEN              1        //松开
#define KEY_DELAY_COUNT_LONG    100      //长按时间     大于1s     单位:10ms
//#define KEY_DELAY_DOUBLE_COUNT  20     //双击时间间隔 小于200ms  单位:10ms



typedef enum
{
    KEY_CH1   = 0x00,
	  KEY_CH2   = 0x01,
	  KEY_CH3   = 0x02,
    KEY_CHN_MAX,
}KEY_CHN_E;

typedef enum
{
    KEY_STA1_Up        = (uint8_t)0x00,  //按键按下
    KEY_STA2_DownShake = (uint8_t)0x01,  //按下抖动
    KEY_STA3_Down      = (uint8_t)0x02,  //按键按下
	  KEY_STA4_UpShake   = (uint8_t)0x03,  //弹起抖动
}KEY_STA_E;


typedef enum
{
    KEY_VAL_NOT                = 0x00,        //没按下
    KEY_VAL_SHORT_CLICK        = 0x01,        //单击
    KEY_VAL_DOUBLE_CLICK       = 0x02,        //双击
    KEY_VAL_SINGLE_LONG_PRESS  = 0x03,        //长按 
    KEY_VAL_LONG_PRESS_LOOSEN  = 0x04,        //长按松开
}KET_VAL_E;




typedef struct
{
    u8  key_sta[KEY_CHN_MAX];   			  //保存按键状态
    u8  key_val[KEY_CHN_MAX];					  //保存按键键值
		u8  key_click[KEY_CHN_MAX];         //保存按键单击->防止丢键
    u8  key_longpress[KEY_CHN_MAX];     //保存按键长按->防止丢键
    u16 key_presstime[KEY_CHN_MAX];	    //保存触发长按的时间(即多久为长按)
	  /*以下为双击检测所需*/
//	u8  key_doubleclick[KEY_CHN_MAX];	  //保存按键双击->防止丢键
//  u8  key_clickflag[KEY_CHN_MAX];     //按键双击标志
//	u8  key_doubletime[KEY_CHN_MAX];    //触发双击的时间(即单击多久内为双击)
}KEY_t;

/***********外部调用函数开始*********/
extern KEY_t g_Key;					//全局的按键结构体变量，可以看做一个对象
void 	 KEY_Init(void);			//按键初始化
void 	 Key_ScanTask(void);	//按键轮询检测状态机函数
//void Key_ScanTask2(void);
/***********外部调用函数结束*********/


				    
#endif
