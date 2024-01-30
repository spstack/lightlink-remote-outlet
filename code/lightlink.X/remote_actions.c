/**
    @brief This contains all the methods for actions that this remote module can perform
    @author Scott Stack
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "config.h"
#include "timer.h"
#include "remote_actions.h"
#include "rfm69/RFM69.h"


/**
 * @brief command handler for load switching command
*/
int cmd_switch_load_handler(remote_cmd_t *cmdbuf)
{
    switch_load_cmd_t *cmd = (switch_load_cmd_t *)cmdbuf;
    int status = NO_ERROR;

    if (cmd == NULL) {
        return ERROR_NULL_PTR;
    }

    // Handle the load switch
    if (cmd->on_off) {
        status = switch_load(cmd->load_id, SWITCH_ON);
        global_data.time_last_pwron_received = timer_get_ticks();
        global_data.poweron_msg_received = true;
    } else {
        status = switch_load(cmd->load_id, SWITCH_OFF);
    }


    return status;
}


/**
 * @brief Switch the specified load ID off
*/
int switch_load(uint8_t load_id, uint8_t on_off)
{
    switch(load_id)
    {
        case LOAD_ID_OUTLET_0:
            global_data.current_switch_val = on_off;
            LOAD0_PIN = on_off;
            break;
        default:
            return ERROR_INVALID_LOAD_ID;
    }

    return NO_ERROR;
}


/**
 * Send command to switch a remote load on or off
*/
int send_load_switch_cmd(uint8_t load_id, uint8_t on_off)
{
    switch_load_cmd_t temp_command;
   
    //construct command
    temp_command.header.magic_num = CMD_MAGIC_BYTE_VAL;
    temp_command.header.opcode = SWITCH_LOAD_OPCODE;
    temp_command.header.bodyLength = sizeof(switch_load_cmd_t) - sizeof(remote_cmd_header_t);
    temp_command.load_id = load_id;
    temp_command.on_off = on_off;
   
    // Send the command!
    bool send_success = RFM69_sendPacket(global_data.current_channel, &temp_command, sizeof(switch_load_cmd_t), true);
   
    if (send_success) {
        global_data.time_last_pwron_sent = timer_get_ticks();
        return NO_ERROR; 
    } else {
        return ERROR_RF_SEND;
    }
}

/**
 * @brief handles all incoming commands
*/
int handle_received_command(uint8_t *cmd_buf, int cmd_length)
{
    int retval = NO_ERROR;

    if (cmd_buf == NULL) {
        return ERROR_NULL_PTR;
    }

    if (cmd_length > sizeof(remote_cmd_t)) {
        return ERROR_INV_BUF_SIZE;
    }

    remote_cmd_t *cmd = (remote_cmd_t *)cmd_buf;

    // Ensure that cmd magic number is correct
    if (cmd->header.magic_num != CMD_MAGIC_BYTE_VAL) {
        return ERROR_CMD_MAGIC_NUM;
    }

    // Handle command depending on what opcode is
    switch(cmd->header.opcode)
    {
        case SWITCH_LOAD_OPCODE:
            retval = cmd_switch_load_handler(cmd);
            break;
        
        default:
            return ERROR_INVALID_OPCODE;
    }

    return retval;
}