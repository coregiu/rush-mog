/**
  ******************************************************************************
  *
  * Define of task manager.
  * author: coregiu
  *
  *
  ******************************************************************************
**/
#include <task_manager.h>
#include "command.h"
#include "controller.h"
#include "motor_direct_executor.h"

// 运动类命令：'0' ~ 'I'（停止/前后左右/转弯/加减速/微调/直行）
// 音频类命令 K~P 不应刷新运动看门狗。
static int is_motion_command(uchar command)
{
    return command >= COMMAND_STOP && command <= COMMAND_DIRECT;
}

void exe_task_from_queue(void *argument)
{
    (void)argument;
    uart_log_string_data("Begin exe task...");
    const TickType_t xTicksToWait = pdMS_TO_TICKS(50);
    const TickType_t motion_timeout = pdMS_TO_TICKS(800);
    struct command_context received_command = {0};
    TickType_t last_motion_command = xTaskGetTickCount();

    for (;;)
    {
        BaseType_t xStatus = xQueueReceive(command_queue, &received_command, xTicksToWait);
        if (xStatus == pdPASS && received_command.cmd_length > 0)
        {
            // 收到停止时丢弃队列里尚未执行的旧运动命令，避免停完又被旧命令带走
            if (received_command.commands[0] == COMMAND_STOP)
            {
                reset_command_queue();
            }

            if (is_motion_command(received_command.commands[0]))
            {
                last_motion_command = xTaskGetTickCount();
            }

            if (received_command.time_sleep_milsec > 0 && received_command.delay_type == DELAY_BEFOR_EXE)
            {
                TickType_t xTicksToDelay = pdMS_TO_TICKS(received_command.time_sleep_milsec);
                vTaskDelay(xTicksToDelay);
            }

            execute_command(&received_command);

            if (received_command.time_sleep_milsec > 0 && received_command.delay_type == DELAY_AFTER_EXE)
            {
                TickType_t xTicksToDelay = pdMS_TO_TICKS(received_command.time_sleep_milsec);
                vTaskDelay(xTicksToDelay);
            }
        }

        // 失败安全：超过 800ms 没有新的运动命令，主动停车。
        // 正常按住方向键时，ESP32 会每 200ms 重发一次运动命令刷新看门狗。
        if ((xTaskGetTickCount() - last_motion_command) > motion_timeout)
        {
            motor_direct_executor.reset();
            last_motion_command = xTaskGetTickCount();
        }
    }
}

void init_freertos()
{
    command_queue = xQueueCreate(MAX_COMMAND_QUEUE_SIZE, sizeof(struct command_context));
    if (command_queue == NULL)
    {
        uart_log_string_data("failed to create command queue");
        return;
    }
    xTaskCreate(exe_task_from_queue, "CONSUMER_TASK", 200, NULL, 2, NULL);
    uart_log_string_data("finish init freertos");
}
