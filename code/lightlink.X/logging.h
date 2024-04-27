/**
 * @file
 * @brief Header file containing all logging methods
 * @author Scott Stack
*/
#include "main.h"


//-------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------
void log_error(error_level_e error_level, uint8_t error_code);
void log_set_log_level(error_level_e err_level);
error_level_e log_get_log_level(void);

void LED_blink_err_code(uint8_t error_code);
void LED_resetIndication(void);
