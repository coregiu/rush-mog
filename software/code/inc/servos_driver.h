/**
  ******************************************************************************
  *
  * the driver of servos
  * author: coregiu
  * 
  * 
  * The command of play: 
  *     RIGHT_LEFT  - turn gimbal left, command 7; 
  *     RIGHT_RIGHT - turn gimbal right, command 8;
  *     RIGHT_TOP - turn gimbal front, command 5; 
  *     RIGHT_DOWN - turn gimbal back, command 6;
  * 
  * The monitor led of this command: P0_4  P0_5 P0_6  P0_7
  * 
  ******************************************************************************
**/

#ifndef SERVOS_DRIVER_GROUP_H
#define SERVOS_DRIVER_GROUP_H

#include <command_receiver.h>
#include <pca9685.h>

enum servo_command_status
{
    HANDLING = 0, IDLE = 1
};

extern const struct module_command_receiver servos_driver;

#endif