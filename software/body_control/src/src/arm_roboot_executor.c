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

const int SERVO_DOWN_RIGHT_POSITION  = 0;
const int SERVO_DOWN_MIDDLE_POSITION = 105;
const int SERVO_DOWN_LEFT_POSITION   = 176;

const int SERVO_TOP_BACK_POSITION    = 30;
const int SERVO_TOP_MIDDLE_POSITION  = 90;
const int SERVO_TOP_FRONT_POSITION   = 160;

// 定义一个初始化函数，同时初始化时基与输出比较
void init_roboot_state(void)
{
    pwm_init();
    // 设置舵机初始状态在90度
    change_angle(CHANNEL_BOTTOM, SERVO_DOWN_MIDDLE_POSITION);
    change_angle(CHANNEL_UP, SERVO_TOP_MIDDLE_POSITION);
}

void update_roboot_state(struct command_context *command_context)
{
    uchar bottom_angle = SERVO_DOWN_MIDDLE_POSITION - gimbal_info.yaw;
    bottom_angle = bottom_angle > SERVO_DOWN_LEFT_POSITION ? SERVO_DOWN_LEFT_POSITION : bottom_angle;
    bottom_angle = bottom_angle < SERVO_DOWN_RIGHT_POSITION ? SERVO_DOWN_RIGHT_POSITION : bottom_angle;
    change_angle(CHANNEL_BOTTOM, bottom_angle);

    uchar top_angle = SERVO_TOP_MIDDLE_POSITION + gimbal_info.roll;
    top_angle = top_angle > SERVO_TOP_FRONT_POSITION ? SERVO_TOP_FRONT_POSITION : top_angle;
    top_angle = top_angle < SERVO_TOP_BACK_POSITION ? SERVO_TOP_BACK_POSITION : top_angle;
    change_angle(CHANNEL_UP, top_angle);
}


const struct module_command_executor arm_roboot_executor = {init_roboot_state, update_roboot_state};
