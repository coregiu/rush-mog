/**
  ******************************************************************************
  *
  * Implement of audio receiver.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <audio_receiver.h>

void init_audio_state()
{

}

char* receive_audio_command()
{
    return "1C";
}



const struct module_command_receiver audio_receiver = {init_audio_state, receive_audio_command};
