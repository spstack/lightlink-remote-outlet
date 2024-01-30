/**
 * @file
 * @brief Header for UART driver
 * @author Scott Stack
*/
#ifndef _UART_H_
#define _UART_H_

#include "main.h"


bool uart_send_string(const char *str, int str_length);
bool uart_send_err_code(uint8_t code);

#endif // _UART_H_