/**
  ******************************************************************************
  *
  * the switch of music
  * author: coregiu
  * 
  * music play switch: P2_6
  * 
  * The command of play: 
  *     RIGHT_TOP  - switch on,  command 5; 
  *     RIGHT_DOWN - switch off, command 6;
  * 
  * The monitor led of this command: P0_4  P0_5
  * 
  * 
  ******************************************************************************
**/

#ifndef MUSIC_SWITCH_H
#define MUSIC_SWITCH_H

#include <command_receiver.h>

#define MUSIC_SWITCH P2_6

extern const struct module_command_receiver music_switch;

#endif