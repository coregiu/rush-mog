/**
  ******************************************************************************
  *
  * the log util
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef UART_LOG_H
#define UART_LOG_H

#define uint unsigned int
#define uchar unsigned char

/**
 * send origin debug data to serial port
 * input log_date: the ascii code of input char
 */
void uart_log_data(uchar log_data);

/**
 * log string
 */
void uart_log_string_data(uchar *log_data);

/**
 * log date to hex code.
 */
void uart_log_hex_data(uchar log_data);

/**
 * send enter key to serial port
 */
void uart_log_enter_char();

/**
 * log coregiu information.
 */
void uart_log_start_info();

#endif