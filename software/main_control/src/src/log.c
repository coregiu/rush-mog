
/**
  ******************************************************************************
  *
  * the log util
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <log.h>

/**
 * send origin debug data to serial port
 * input log_date: the ascii code of input char
 */
void uart_log_data(uchar log_data)
{

}

void uart_log_string_data(uchar *log_data)
{
    // int size = sizeof(log_data) / sizeof(log_data[0]);
    // for (int i = 0; i < size; i++)
    // {
    //     uart_log_data(log_data[i]);
    // }
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
 * log date to hex code.
 */
void uart_log_hex_data(uchar log_data)
{
	// int converted_data;
	// uart_log_data(0x30);
	// uart_log_data(0x78);
	// int log_hex_data = log_data / 16;
	// log_hex_data = log_data % 16;
	// if (log_hex_data % 16 < 10)
	// {
	// 	converted_data = log_hex_data % 16 + 0x30;
	// }
	// else
	// {
	// 	converted_data = log_hex_data % 16 + 0x37;
	// }
	// uart_log_data(converted_data);
	// uart_log_enter_char();
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
    // uchar start_info[20] = {'s','t','a','r','t',' ','s','u','c','e','s','s','@','c','o','r','e','g','i','u'};
    // for (int i = 0; i < 37; i++)
    // {
    //     uart_log_data('-');
    // }
    // uart_log_enter_char();

    // uart_log_data('-');
    // uart_log_data('-');
    // uart_log_data('-');
    // uart_log_data(' ');
    // uart_log_data(' ');
    // for (int i = 0; i < 20; i++)
    // {
    //     uart_log_data(start_info[i]);
    // }
    // uart_log_data(' ');
    // uart_log_data(' ');
    // uart_log_data('-');
    // uart_log_data('-');
    // uart_log_data('-');

    // uart_log_enter_char();
    // for (int i = 0; i < 37; i++)
    // {
    //     uart_log_data('-');
    // }
}