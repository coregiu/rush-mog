/**
  ******************************************************************************
  *
  * Define of pwm controller.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef DRIVER_OF_PWM_H
#define DRIVER_OF_PWM_H

#include "command.h"

enum pwm_channel
{
    CHANNEL_BOTTOM = '1',
    CHANNEL_UP     = '2'
};

void pwm_init();

// angle: 0-180
void change_angle(enum pwm_channel channel, uchar angle);

#endif