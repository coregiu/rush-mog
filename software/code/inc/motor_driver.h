/**
  ******************************************************************************
  *
  * The driver of motor
  * author: coregiu
  * 
  * MOVE command:
  *     key: LEFT_TOP
  *     monitor led: P0_0
  *     gpio:
  *         LEFT_EN = 1
  *         LEFT_MV = 1
  *         LEFT_BK = 0
  *         RIGHT_EN = 1
  *         RIGHT_MV = 1
  *         RIGHT_BK = 0
  * 
  * BACK command:
  *     key: LEFT_DOWN
  *     monitor led: P0_1
  *     gpio:
  *         LEFT_EN = 1
  *         LEFT_MV = 0
  *         LEFT_BK = 1
  *         RIGHT_EN = 1
  *         RIGHT_MV = 0
  *         RIGHT_BK = 1
  * 
  * TURN LEFT command:
  *     key: LEFT_LEFT
  *     monitor led: P0_2
  *     gpio:
  *         LEFT_EN = 0
  *         LEFT_MV = 0
  *         LEFT_BK = 0
  *         RIGHT_EN = 1
  *         RIGHT_MV = 1
  *         RIGHT_BK = 0
  * 
  * TURN RIGHT command:
  *     key: LEFT_RIGHT
  *     monitor led: P0_3
  *     gpio:
  *         LEFT_EN = 1
  *         LEFT_MV = 1
  *         LEFT_BK = 0
  *         RIGHT_EN = 0
  *         RIGHT_MV = 0
  *         RIGHT_BK = 0
  * 
  * P2_0 : RIGHT_EN  L298N_EnA
  * P2_1 : RIGHT_MV  L298N_IN2
  * P2_2 : RIGHT_BK  L298N_IN1 
  * P2_3 : LEFT_EN   L298N_EnB
  * P2_4 : LEFT_MV   L298N_IN4
  * P2_5 : LEFT_BK   L298N_IN3
  * 
  ******************************************************************************
**/
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <command_receiver.h>

#define   RIGHT_EN   P2_0
#define   RIGHT_MV   P2_1
#define   RIGHT_BK   P2_2
#define   LEFT_MV    P2_3
#define   LEFT_BK    P2_4
#define   LEFT_EN    P2_5

// CAR run state
enum car_run_state
{
    STOP = 0, MOVE = 1, BACK = 2, LEFT = 3, RIGHT = 4
};

extern const struct module_command_receiver motor_driver;

#endif