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

void init_freertos()
{
    command_queue = xQueueCreate(MAX_COMMAND_QUEUE_SIZE, sizeof(struct command_des));
    if (command_queue == NULL)
    {
        uart_log_string_data("failed to create command queue");
        uart_log_enter_char();
        return;
    }
    xTaskCreate(exe_task_from_queue, "CONSUMER_TASK", 200, NULL, 3, NULL);
}

void exe_task_from_queue(void *argument)
{
    const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
    struct command_des received_command = {0, '0'};

    for (;;)
    {
        //通过接收函数从xTemperatureQueue队列中获取温度数据
        BaseType_t xStatus = xQueueReceive(command_queue, &received_command, xTicksToWait);
        if (xStatus == pdPASS)
        {
            execute_commands(&received_command.command);
            if (received_command.time_sleep_milsec > 0)
            {
                TickType_t xTicksToDelay = pdMS_TO_TICKS(received_command.time_sleep_milsec);
                vTaskDelay(xTicksToDelay);
            }
        }
    }
}