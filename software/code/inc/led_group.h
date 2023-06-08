/**
  ******************************************************************************
  *
  * the switch of leds
  * author: coregiu
  * 
  * led light switch: P1_2； GND： P1_3
  * 
  * The command of play: 
  *     RIGHT_LEFT  - switch on,  command 11; 
  *     RIGHT_RIGHT - switch off, command 12;
  * 
  * The monitor led of this command: P0_6  P0_7
  * 
  ******************************************************************************
**/

#ifndef MUSIC_LED_GROUP_H
#define MUSIC_LED_GROUP_H

#include <command_receiver.h>

#define  LED_ABOVE_CAR  P1_2
#define  GND_ABOVE_CAR  P1_3

extern const struct module_command_receiver led_group;

#endif