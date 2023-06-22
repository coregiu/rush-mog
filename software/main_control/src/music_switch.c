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

#include <music_switch.h>
#include <mcs51/8051.h>

void init_music_switch()
{
    MUSIC_SWITCH = SWITCH_OFF;
}

void music_switch_on()
{
    if (MUSIC_SWITCH != SWITCH_ON)
    {
        MUSIC_SWITCH = SWITCH_ON;
    }
}

void music_switch_off()
{
    if (MUSIC_SWITCH != SWITCH_OFF)
    {
        MUSIC_SWITCH = SWITCH_OFF;
    }
}

void update_music_state(uint car_cmd)
{
    if (car_cmd == COMMAND_RIGHT_TOP)
    {
        // uart_log_string_data("e:5");// send 5
        LED_RIGHT_TOP = !LED_RIGHT_TOP;
        music_switch_on();
    }
    else if (car_cmd == COMMAND_RIGHT_DOWN) 
    {
        // uart_log_string_data("e:6"); // send 6
        LED_RIGHT_DOWN = !LED_RIGHT_DOWN;
        music_switch_off();
    }
}

const struct module_command_receiver music_switch = {init_music_switch, update_music_state};