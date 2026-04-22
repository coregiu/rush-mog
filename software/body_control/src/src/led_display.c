/**
  ******************************************************************************
  *
  * Define of led display executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <led_display.h>

const uchar PIXEL_DISTANCE_INTERVAL = 6;

void init_led_display()
{
    OLED_Init();            //初始化OLED
    show_gimbal_info(&gimbal_info);
}

void display_info(struct command_context *command_context)
{
    show_gimbal_info(&gimbal_info);
}

void show_short_number(u8 x, u8 y, short number)
{
    if (number == 0)
    {
        OLED_ShowChar(x, y, 0x30, 8);
        return;
    }

    short loop_number = number;
    if (number < 0)
    {
        OLED_ShowChar(x, y, '-', 8);
        loop_number = ~(number - 1);
        x += PIXEL_DISTANCE_INTERVAL;
    }

    OLED_ShowShort(&x, &y, loop_number);
}

void show_gimbal_info(struct gimbal_info *gimbal_info)
{
    OLED_Clear();

    // OLED_ShowCHinese(54, 0, 3);  //电
    // OLED_ShowCHinese(72, 0, 4);  //子
    // OLED_ShowCHinese(90, 0, 5);  //科
    // OLED_ShowCHinese(108, 0, 6); //技
    OLED_ShowString(40, 0, "GIMBAL", 16);
    OLED_ShowString(19, 2, "Zhang ZiYan", 8);

    OLED_ShowString(27, 4, "Rol: ", 8);
    OLED_ShowString(27, 5, "Pic: ", 8);
    OLED_ShowString(27, 6, "Yaw: ", 8);
    OLED_ShowString(27, 7, "Tmp: ", 8);

    show_short_number(60, 4, gimbal_info->roll);
    show_short_number(60, 5, gimbal_info->pitch);
    show_short_number(60, 6, gimbal_info->yaw);
    show_short_number(60, 7, gimbal_info->temperature);
}

const struct module_command_executor led_display_executor = {init_led_display, display_info};
