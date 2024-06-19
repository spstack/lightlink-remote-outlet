/**
 * @file
 * @brief Source file containing all logging methods
 * @author Scott Stack
*/
#include "main.h"
#include "config.h"
#include "uart.h"
#include "logging.h"
#include "timer.h"

// setup global to track current log level - this will generally be overwritten with DEFAULT_LOG_LEVEL in main.h
static error_level_e _curr_err_level = CRITICAL_LEVEL;



/**
 * @brief Log that an error has occurred. This can perform different actions depending on the configuration
 * @param[in] error_code - the error code to log
*/
void log_error(error_level_e error_level, uint8_t error_code)
{
    if (log_get_log_level() <= error_level)
    {
#if TRAP_ERROR
        while (1) {
#endif
#ifdef DEBUG_UART_OUT
        uart_send_err_code(error_code);
#endif

#ifdef DEBUG_LED_BLINK
        LED_blink_err_code(error_code);
#endif
#if TRAP_ERROR
        CLRWDT();
        }
#endif

    }

}


/**
 * @brief Set the current log level
*/
void log_set_log_level(error_level_e err_level)
{
    _curr_err_level = err_level;
}


/**
 * @brief Get the current log level
*/
error_level_e log_get_log_level(void)
{
    return _curr_err_level;
}


/**
 * @brief Code to blink our one LED with an error code specified by 'pattern'
 * @details Each bit in pattern represents a double or single LED pulse.
 * Single Pulse = 0
 * Double Pulse = 1
*/
void LED_blink_err_code(uint8_t error_code)
{
    uint8_t i, current_bit;
    const uint32_t blink_bit_period_ms = 750;
    const uint32_t double_blink_time_ms = 100;
    const uint32_t single_blink_time_ms = 250;

    // Clear WDT to ensure it doesn't expire while we do this long operation
    CLRWDT();

    // Turn pin off initially to indicate error code incoming
    DEBUG_LED_PIN = 0;
    timer_wait_ms(1000);
    
    // for each bit in 'pattern' blink a long or short pulse
    for(i = 0; i < 8; i++)
    {
        current_bit = (error_code >> i) & 0x1;
        
        if (current_bit)
        {
            // if '1' then do a quick double pulse
            DEBUG_LED_PIN = 1;
            timer_wait_ms(double_blink_time_ms);
            DEBUG_LED_PIN = 0;
            timer_wait_ms(double_blink_time_ms);
            DEBUG_LED_PIN = 1;
            timer_wait_ms(double_blink_time_ms);
            DEBUG_LED_PIN = 0;
            timer_wait_ms(double_blink_time_ms);

            // Wait the rest of the period
            timer_wait_ms(blink_bit_period_ms - (4 * double_blink_time_ms));
        }
        else
        {
            // if zero, then single pulse
            DEBUG_LED_PIN = 1;
            timer_wait_ms(single_blink_time_ms);
            DEBUG_LED_PIN = 0;
            timer_wait_ms(single_blink_time_ms);   

            // Wait the rest of the period
            timer_wait_ms(blink_bit_period_ms - (2 * single_blink_time_ms));
        }

        CLRWDT();
    }

    // return pin to 1 
    timer_wait_ms(1000);
    DEBUG_LED_PIN = 1;

    CLRWDT();
}

/**
 * @brief - blink LEDs in a specific pattern to indicate reset
*/
void LED_resetIndication(void)
{
    // Flash LED to indicate reset
    DEBUG_LED_PIN = 1;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 0;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 1;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 0;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 1;
}


