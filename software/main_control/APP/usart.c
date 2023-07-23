#include "usart.h"
#include "stdarg.h"
#include "String.h"
#include "ringbuff.h"

#define DATA_RX_BUFSIZE 256

static ringbuff_t s_recieve_data_ring_buf;
static uint8_t s_rx_buf[DATA_RX_BUFSIZE];

/*
*********************************************************************************************************
*	函 数 名: ring_buffer_init
*	功能说明: 初始化环形缓冲区
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void ring_buffer_init(void)
{
  ringbuff_init(&s_recieve_data_ring_buf,s_rx_buf,DATA_RX_BUFSIZE);
}

/*
*********************************************************************************************************
*	函 数 名: uart1_NvicConfig
*	功能说明: 串口1NVIC设置
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void uart1_NvicConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		    //分组4下无子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			    //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}


/*
*********************************************************************************************************
*	函 数 名: uart1_Init
*	功能说明: 串口1初始化函数
*	形    参: 1.bound（串口波特率）
*	返 回 值: 无
*********************************************************************************************************
*/ 
void uart1_Init(u32 bound)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	uart1_NvicConfig();  //NVIC配置
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //开启串口接收中断
//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  //开启串口空闲中断
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	//USART_ClearFlag(USART1, USART_FLAG_TC);   //解决第一个字节丢失问题,如果第一个字节丢失可以加上这行
	ring_buffer_init();    //初始化环形缓冲区
}


//加入以下代码,支持printf函数,而不需要选择use MicroLIB	
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;  

//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

/*
*********************************************************************************************************
*	函 数 名: fputc
*	功能说明: 重定义fputc函数
*	形    参: 1:ch(待写入字符) 2:f(文件指针)
*	返 回 值: 无
*********************************************************************************************************
*/ 
#define USE_REGISTER      //是否使用寄存器操作

int fputc(int ch, FILE *f)
{ 	

#ifdef 	USE_REGISTER      //使用寄存器操作

	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;  
	
#else	                    //使用库函数操作
	
	USART_SendData(USART1, (unsigned char)ch);
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TC));
    USART_ClearFlag(USART1, USART_FLAG_TC);
	
#endif
	
	return ch;
}


/*
*********************************************************************************************************
*	函 数 名: Printf
*	功能说明: 可变参数重定义Printf函数
*	形    参: 1.串口号 2.char* fmt,...  格式化输出字符串和参数
*	返 回 值: 无
*********************************************************************************************************
*/ 
#define    USART_SENDBUFFLEN     256          //定义打印的字符串数组长度

char g_USART_SendBuff[USART_SENDBUFFLEN];     //用于打印的字符串数组

void Printf(USART_TypeDef* USARTx, char* fmt, ...)
{
   
		u16 i = 0;                                //清空下标
		memset(g_USART_SendBuff,0,USART_SENDBUFFLEN); //清空字符数组
	
		va_list ap;                               //可变参数列表变量（对象）
		va_start(ap,fmt);                         //指明char*类型的fmt参数后面的才是可变参数的开始
		vsprintf(g_USART_SendBuff,fmt,ap);        //使用参数列表发送格式化输出到字符串
		va_end(ap);	                              //清空参数列表变量（对象）

	    while(g_USART_SendBuff[i])                //如果字符串数组为NULL则退出打印
		{
		  while((USARTx->SR&0X40)==0);
		  USARTx->DR = g_USART_SendBuff[i++];
		}
}


/*
*********************************************************************************************************
*	函 数 名: UartSendBuff
*	功能说明: 串口1发送函数
*	形    参: 要发送的数据  和  数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void UartSendBuff(u8 * buff, u16 len)
{
	while(len--)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET); //等待发送结束
		USART_SendData(USART1, *buff++);
	}
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
}

/*
*********************************************************************************************************
*	函 数 名: USART1_IRQHandler
*	功能说明: 串口1接收中断服务函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 

void USART1_IRQHandler(void)  //串口1中断服务程序
{
	uint8_t receive_data;
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET) //接收中断触发
	{		
		receive_data = USART_ReceiveData(USART1); 
		//USART_SendData(USART1,receive_data);
		//写入环形缓冲区
		ringbuff_write(&s_recieve_data_ring_buf,&receive_data,1);
	}	
}

/*
*********************************************************************************************************
*	函 数 名: uart1_process
*	功能说明: 数据处理函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void uart1_process(void)
{
	uint8_t buf[DATA_RX_BUFSIZE] = {0};
	uint32_t datalen = 0;
	datalen = ringbuff_get_linear_block_length(&s_recieve_data_ring_buf);
	if(datalen != 0)
	{
		ringbuff_read(&s_recieve_data_ring_buf,buf,datalen);
		UartSendBuff(buf,datalen);
	}
}

