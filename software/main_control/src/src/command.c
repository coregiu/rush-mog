/**
  ******************************************************************************
  *
  * Define of command, receiver and executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <command.h>

QueueHandle_t command_queue;

// convert command to the array sequence of command_module_map.
uint convert_command_seq(char command)
{
    if (command >= '0' && command <= '9')
    {
        return command - '0';
    }
    else if (command >= 'A' && command < 'Z')
    {
        return command - 'A' + 10;
    }
    else
    {
        return COMMANDS_LENGTH - 1;
    }
}

void init_command_module()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA,PB, PC端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;        //LED-->PC13 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度为50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);            //根据设定参数初始化GPIOC.13
    GPIO_SetBits(GPIOC, GPIO_Pin_13);                 //PC.13 输出高
}
