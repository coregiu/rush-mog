#ifndef __DBG_H__
#define __DBG_H__

#include "stm32f4xx.h"



/*
**********************************************************************************************************
											打印相关开始
**********************************************************************************************************
*/

#define USER_DEBUG  		 //启用打印 ->用于代码控制

#ifdef  USER_DEBUG
#define Println(format,...)   						\
{ 							  												\
	if(Print_Switch)												\
	{       				                        \
		printf(format "\r\n",##__VA_ARGS__); \
	}                                       \
}                                         
#else
    #define Println(format,...)          
#endif

#ifdef  USER_DEBUG
#define PrintlnErr(format,...)   					\
{ 							  												\
	if(Print_Switch)												\
	{       				                        \
		printf("[%-10s](%d) : " format "\r\n",__func__,__LINE__,##__VA_ARGS__); \
	}                                       \
}                                         
#else
    #define PrintlnErr(format,...)          
#endif
/*
**********************************************************************************************************
											打印相关结束
**********************************************************************************************************
*/

extern u8 Print_Switch;


#endif

