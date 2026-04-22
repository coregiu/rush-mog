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

#include <stdlib.h>
#include "stm32f10x.h"

// #define uint unsigned int
#define uchar unsigned char

/**
 * send origin debug data to serial port
 * input log_date: the ascii code of input char
 */
void uart_log_data(uint16_t log_data);

/**
 * log string with enter
 */
void uart_log_string_data(char *log_data);

/**
 * log string without enter
 */
void uart_log_string_no_enter(char *log_data);

/**
 * send enter key to serial port
 */
void uart_log_enter_char();

/**
 * log coregiu information.
 */
void uart_log_start_info();

/**
 * count length of string array.
 */
uint16_t count_str(char *str);

void uart_log_number(int log_number);

#endif