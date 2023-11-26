/**
  ******************************************************************************
  *
  * Define of vehicle executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef EXECUTOR_VEHICLE_H
#define EXECUTOR_VEHICLE_H

#include "command.h"

#define IN1 PAout(11) // PA11
#define IN2 PAout(10) // PA10
#define IN3 PAout(9)  // PA9
#define IN4 PAout(8)  // PA8
#define IN5 PBout(15) // PB15
#define IN6 PBout(14) // PB14
#define IN7 PBout(13) // PB13
#define IN8 PBout(12) // PB12

#define GPIO_IN1 GPIO_Pin_11 // PA11
#define GPIO_IN2 GPIO_Pin_10 // PA10
#define GPIO_IN3 GPIO_Pin_9  // PA9
#define GPIO_IN4 GPIO_Pin_8  // PA8
#define GPIO_IN5 GPIO_Pin_15 // PB15
#define GPIO_IN6 GPIO_Pin_14 // PB14
#define GPIO_IN7 GPIO_Pin_13 // PB13
#define GPIO_IN8 GPIO_Pin_12 // PB12

// vehicle run state
enum vehicle_state
{
    STOP        = 0,
    MOVE        = 1,
    BACK        = 2,
    LEFT        = 3,
    RIGHT       = 4,
    LEFT_FRONT  = 5,
    RIGHT_FRONT = 6,
    LEFT_BACK   = 7,
    RIGHT_BACK  = 8,
    LEFT_TURN   = 9,
    RIGHT_TURN  = 10,
    TURN_OVER   = 11
};

extern const struct module_command_executor vehicle_executor;
#endif