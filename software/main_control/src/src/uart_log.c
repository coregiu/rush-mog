
/**
  ******************************************************************************
  *
  * the log util
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <uart_log.h>

/**
 * send origin debug data to serial port
 * input log_date: the ascii code of input char
 */
void uart_log_data(uint16_t log_data)
{
    USART_SendData(USART2, log_data);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == 0);
    USART_ClearFlag(USART2, USART_FLAG_TC);
}

void uart_log_string_data(char *log_data)
{
    uint16_t size = count_str(log_data);
    for (uint16_t i = 0; i < size; i++)
    {
        uart_log_data(log_data[i]);
    }
    uart_log_enter_char();
}

/**
 * log enter key
 */
void uart_log_enter_char()
{
    uart_log_data(0x0d);
    uart_log_data(0x0a);
}

/**
 * print:
 * ------------------------------
 * ---  start sucess@coregiu  ---
 * ------------------------------
 *
*/
void uart_log_start_info()
{
    uint16_t start_info[20] = {'S', 't', 'a', 'r', 't', ' ', 's', 'u', 'c', 'e', 's', 's', '@', 'c', 'o', 'r', 'e', 'g', 'i', 'u'};
    for (uint16_t i = 0; i < 37; i++)
    {
        uart_log_data('-');
    }
    uart_log_enter_char();

    uart_log_data('-');
    uart_log_data('-');
    uart_log_data('-');
    uart_log_data(' ');
    uart_log_data(' ');
    for (uint16_t i = 0; i < 20; i++)
    {
        uart_log_data(start_info[i]);
    }
    uart_log_data(' ');
    uart_log_data(' ');
    uart_log_data('-');
    uart_log_data('-');
    uart_log_data('-');

    uart_log_enter_char();
    for (uint16_t i = 0; i < 37; i++)
    {
        uart_log_data('-');
    }
    uart_log_enter_char();
}

uint16_t count_str(char *str)
{
    int i = 0;
    while (str[i++] != '\0')
        ;
    return i;
}