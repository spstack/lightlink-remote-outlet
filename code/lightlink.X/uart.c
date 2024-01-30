/**
 * @file
 * @brief UART driver for the PIC16F18146
 * @author Scott Stack
*/
#include <xc.h>
#include "config.h"
#include "uart.h"


/**
 * @brief Send string over debug UART console
*/
bool uart_send_string(const char *str, int str_length)
{
    for (int i = 0; i < str_length; i++)
    {
        // Wait for transmit ready
        while(TX1STAbits.TRMT == 0);
        TX1REG = str[i]; 
    }
    return true;
}

/**
 * @brief Send error code out on UART console
*/
bool uart_send_err_code(uint8_t code)
{
    while(TX1STAbits.TRMT == 0);
    TX1REG = code;
    return true;
}


