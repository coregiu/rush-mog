#include "Key.h"

/*声明一个按键类型的结构体*/
KEY_t g_Key = {0};



/*
*********************************************************************************************************
*	函 数 名: KEY_Init
*	功能说明: 按键初始化 初始化GPIOE_10,11,12为上拉输入引脚
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void KEY_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//使能PORTE时钟
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;//PE10,11,12
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; //设置成输入
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; //设置下拉
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE
} 


/*
*********************************************************************************************************
*	函 数 名: Key_GetIoState
*	功能说明: 按键引脚状态获取
*	形    参: 1.按键编号
*	返 回 值: 引脚电平（高电平为:1,低电平为:0）
*********************************************************************************************************
*/ 
u8 Key_GetIoState(u8 id)
{
    u8 val = 0;
    switch(id)
    {
        case KEY_CH1:
            val = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10);
            break;
				
        case KEY_CH2:
            val = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11);
            break;
				
        case KEY_CH3:
            val = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12);
            break;
    }
    return val;
}



/*
*********************************************************************************************************
*	函 数 名: Key_ScanTask
*	功能说明: 按键轮询状态机
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void Key_ScanTask(void)
{
    static u32 update_10ms = 0;       	//静态变量，用于与时基进行比较，达到软延迟的效果
    if(SysTick_1ms - update_10ms > 10)  //10ms进来一次，也有消除抖动的作用
    {
        update_10ms = SysTick_1ms;   //更新时间
        for(u8 i=0; i<KEY_CHN_MAX; i++)
        {
            switch(g_Key.key_sta[i])
            {
                case KEY_STA1_Up:       //按键弹起状态
                    if( Key_GetIoState(i) == KEY_PRESS )   //检测到按键按下的电平
                    {          
												g_Key.key_sta[i] = KEY_STA2_DownShake;   //跳到按键抖动状态
                    }
                    else                                   //没有检测到按键按下
                    {
                        g_Key.key_sta[i] = KEY_STA1_Up;   //留在当前状态
                        g_Key.key_val[i] = KEY_VAL_NOT;   //键值为无按键按下
                    }
                  break;
					
                case KEY_STA2_DownShake:      //按键抖动状态
                    if( Key_GetIoState(i) == KEY_PRESS )   //检测到按键还是按下的电平
                    {  
												g_Key.key_sta[i] = KEY_STA3_Down;  //跳转到按键按下状态
												g_Key.key_presstime[i] = 0;        //按键按下后要进行长按检测，所以长按时间清零
										}	
										else
										{
												g_Key.key_sta[i] = KEY_STA1_Up;   //跳转到按键弹起状态
										}
									break;
					
							  case KEY_STA3_Down:           //按键按下状态
										g_Key.key_presstime[i]++;          //长按计时时间++;
										if(g_Key.key_presstime[i] < KEY_DELAY_COUNT_LONG)  //长按时间小于2秒
										{
											if(Key_GetIoState(i) == KEY_LOOSEN)          //按键松开
											{
												g_Key.key_val[i] = KEY_VAL_SHORT_CLICK;   //键值为按键单击	
												g_Key.key_click[i] = true;                //按键单击
												g_Key.key_sta[i] = KEY_STA4_UpShake;   	  //跳到到按键弹起抖动状态													
											}
										}
										else           //长按时间大于2秒，则为长按
										{
											if(Key_GetIoState(i) == KEY_LOOSEN)              //按键松开
											{
												 g_Key.key_val[i] = KEY_VAL_LONG_PRESS_LOOSEN; //键值为长按松开
												 g_Key.key_sta[i] = KEY_STA4_UpShake;   	     //跳到到按键弹起抖动状态
											}
											else 
											{
												 if(g_Key.key_val[i] != KEY_VAL_SINGLE_LONG_PRESS)
												 {
													 g_Key.key_longpress[i] = true;                //按键长按											 
												 }
												 g_Key.key_val[i] = KEY_VAL_SINGLE_LONG_PRESS; //键值为长按状态 
											}					
										}								
                  break;
				
								case KEY_STA4_UpShake:		  //按键弹起抖动
										if(Key_GetIoState(i) == KEY_LOOSEN)   //按键是松开的
										{
											 g_Key.key_sta[i] = KEY_STA1_Up;   	       //跳到到按键弹起状态
											 g_Key.key_val[i] = KEY_VAL_NOT;           //没有按键按下状态
										}
//										else   									//如果只是按键抖动
//										{
//										    g_Key.key_sta[i] = KEY_STA3_Down;   	   //跳回到按键按下状态
//										}
								 break;
							default:  g_Key.key_sta[i] = KEY_STA1_Up;   	 //跳到到按键弹起状态
            } 
        }
    }
}

//支持双击
//void Key_ScanTask1(void)
//{
//    static u32 update_10ms = 0; 
//    
//    if(SysTick_1ms - update_10ms > 10)
//    {
//        update_10ms = SysTick_1ms;   //更新时间
//        
//        for(u8 i=0; i<KEY_CHN_MAX; i++)
//        {
//            switch(g_Key.key_sta[i])
//            {
//                case KEY_STA1_Up:       //按键弹起状态
//                    if( Key_GetIoState(i) == KEY_PRESS )   //检测到按键按下的电平
//                    {           
//											g_Key.key_sta[i] = KEY_STA2_DownShake;   //跳到按键抖动状态
//                    }
//                    else                                   //没有检测到按键按下
//                    {
//											if(g_Key.key_clickflag[i] == 1)    //有单击标志
//											{
//												 g_Key.key_doubletime[i] ++;     //双击时间间隔++
//												 if(g_Key.key_doubletime[i] >=  KEY_DELAY_DOUBLE_COUNT)  //超过双击时间则为单击
//												 {
//													g_Key.key_clickflag[i] = 0;               //清除单击标志
//													g_Key.key_val[i] = KEY_VAL_SHORT_CLICK;   //键值为单击	
//													g_Key.key_click[i] = true;                //按键单击
//												 }
//												
//											}
//											else
//											{
//												g_Key.key_sta[i] = KEY_STA1_Up;   //留在当前状态
//												g_Key.key_val[i] = KEY_VAL_NOT;   //键值为无按键按下						
//											}

//                    }
//					
//                    break;
//					
//                case KEY_STA2_DownShake:      //按键抖动状态
//                    if( Key_GetIoState(i) == KEY_PRESS )   //检测到按键还是按下的电平
//                    {  
//												g_Key.key_sta[i] = KEY_STA3_Down;  //跳转到按键按下状态
//												g_Key.key_presstime[i] = 0;        //按键按下后要进行长按检测，所以长按时间清零
//										}	
//										else
//										{
//												g_Key.key_sta[i] = KEY_STA1_Up;   //跳转到按键弹起状态
//										}
//                    break;
//                                    
//                case KEY_STA3_Down:           //按键按下状态
//									g_Key.key_presstime[i]++;          //长按计时时间++;
//									if(g_Key.key_presstime[i] < KEY_DELAY_COUNT_LONG)  //长按时间小于2秒
//									{
//										if(Key_GetIoState(i) == KEY_LOOSEN)           //按键松开
//										{
//											if(g_Key.key_clickflag[i] == 1)            //有单击的标志
//											{ 
//													g_Key.key_clickflag[i] = 0;               //单击标志清空
//													g_Key.key_val[i] = KEY_VAL_DOUBLE_CLICK;  //键值为双击
//														g_Key.key_doubleclick[KEY_CH1] = true;    //按键双击
//													g_Key.key_sta[i] = KEY_STA4_UpShake;   	  //跳到到按键弹起抖动状态								
//											}
//											else
//											{
//												g_Key.key_clickflag[i] = 1;               //单击标志置一
//												g_Key.key_doubletime[i] = 0;              //清空双击时间间隔
//												g_Key.key_sta[i] = KEY_STA4_UpShake;   	  //跳到到按键弹起抖动状态								
//											}						
//										}
//									}
//									else           //长按时间大于2秒，则为长按
//									{
//										if(Key_GetIoState(i) == KEY_LOOSEN)              //按键松开
//										{
//											 g_Key.key_val[i] = KEY_VAL_LONG_PRESS_LOOSEN; //键值为长按松开
//											 g_Key.key_sta[i] = KEY_STA4_UpShake;   	     //跳到到按键弹起抖动状态
//										}
//										else 
//										{ 
//											 if(g_Key.key_val[i] != KEY_VAL_SINGLE_LONG_PRESS)
//											 {
//												 g_Key.key_longpress[i] = true;
//											 }
//											 g_Key.key_val[i] = KEY_VAL_SINGLE_LONG_PRESS; //键值为长按状态
//											 
//											
//										}		
//									}			
//          break;
//				
//				case KEY_STA4_UpShake:		  //按键弹起抖动
//						if(Key_GetIoState(i) == KEY_LOOSEN)   //按键是松开的
//						{
//							 g_Key.key_sta[i] = KEY_STA1_Up;   	     //跳到到按键弹起状态

//						}
//					
//					 break;
//				default:  g_Key.key_sta[i] = KEY_STA1_Up;   	 //跳到到按键弹起状态
//        } 
//      }
//    }
//}


