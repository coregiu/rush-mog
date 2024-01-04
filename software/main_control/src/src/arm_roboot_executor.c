/**
  ******************************************************************************
  *
  * Define of arm roboot executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <arm_roboot_executor.h>
#include "stm32_pca9685.h"

void init_roboot_state()
{
    pca_servo_init(SERVO_1, 60, SERVO_RANGE_1_INIT);
    pca_servo_init(SERVO_2, 60, SERVO_RANGE_2_INIT);
    pca_servo_init(SERVO_3, 60, SERVO_RANGE_3_INIT);
    pca_servo_init(SERVO_4, 60, SERVO_RANGE_4_INIT);
    pca_servo_init(SERVO_5, 60, SERVO_RANGE_5_INIT);
    pca_servo_init(SERVO_6, 60, SERVO_RANGE_6_INIT);
}

void update_roboot_state(struct command_context *command_context)
{

}

const struct module_command_executor arm_roboot_executor = {init_roboot_state, update_roboot_state};
