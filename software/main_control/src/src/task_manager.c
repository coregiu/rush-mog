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

void exe_task_from_queue(void *argument)
{
    uart_log_string_data("Begin exe task...");
    const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
    struct command_context received_command = {0, DELAY_BEFOR_EXE, '0'};

    for (;;)
    {
        //通过接收函数从xTemperatureQueue队列中获取温度数据
        BaseType_t xStatus = xQueueReceive(command_queue, &received_command, xTicksToWait);
        if (xStatus != pdPASS)
        {
            continue;
        }
        if (received_command.delay_type == DELAY_BEFOR_EXE && received_command.time_sleep_milsec > 0)
        {
            TickType_t xTicksToDelay = pdMS_TO_TICKS(received_command.time_sleep_milsec);
            vTaskDelay(xTicksToDelay);
        }
        // uart_log_data(received_command.command);
        execute_commands(&received_command.command, COMMAND_TYPE_AUTO);

        if (received_command.delay_type == DELAY_AFTER_EXE && received_command.time_sleep_milsec > 0)
        {
            TickType_t xTicksToDelay = pdMS_TO_TICKS(received_command.time_sleep_milsec);
            vTaskDelay(xTicksToDelay);
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