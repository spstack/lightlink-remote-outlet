
#include <xc.h>
#include "main.h"
#include "config.h"
#include "remote_actions.h"
#include "rfm69/RFM69.h"



/**
 * @brief main interrupt handler
*/
void __interrupt () main_isr(void)
{
    // TIMER 0 SYSTEM TICK interrupt @ 1ms
    if (TMR0IF && TMR0IE)
    {
        TMR0IF = 0;
    
        //init timer 0 so overflow happens in 1ms.
        // TMR0 = TIMER_INIT_VALUE_1MS;
        SYSTEM_TICKS += 1;
    }
    
    // If any other IOC flags are set, clear them
    if (IOCIF && IOCIE)
    {
        // Clear flag
        IOCIF = 0;
    }
    
    // Check for power loss indication 
    if (PLI_INT_FLAG)
    {
        volatile int8_t dummy = 1;
        switch_load_cmd_t temp_command;

        // Clear watchdog once before we go down to ensure we don't prematurely reset in the event
        // that this is a power glitch or we lose 12V slowly
        CLRWDT();

        // construct load switch command
        temp_command.header.magic_num = CMD_MAGIC_BYTE_VAL;
        temp_command.header.opcode = SWITCH_LOAD_OPCODE;
        temp_command.header.bodyLength = sizeof(switch_load_cmd_t) - sizeof(remote_cmd_header_t);
        temp_command.load_id = LOAD_ID_OUTLET_0;
        temp_command.on_off = SWITCH_OFF;
   
        // First turn off our own switch to conserve energy
        switch_load(LOAD_ID_OUTLET_0, SWITCH_OFF);

        // Next send several switch off commands to the other terminal in quick succession
        for (int8_t i = 0; i < 10; i++) {
            (void)RFM69_sendFrame(global_data.current_channel, &temp_command, sizeof(switch_load_cmd_t), false, false);
        }
        
        // Spin forever until we lose power or watchdog resets us...
        while(dummy);
    }
}

