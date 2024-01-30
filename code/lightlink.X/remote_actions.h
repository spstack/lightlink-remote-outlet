/* 
 * File:   remote_actions.h
 * Author: Scott
 *
 * Created on March 18, 2018, 5:11 PM
 */

#ifndef REMOTE_ACTIONS_H
#define	REMOTE_ACTIONS_H

#ifdef	__cplusplus
extern "C" {
#endif

//-------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------

#define MAX_BODY_SIZE       55      ///< This is determined by max size of packet in RFM69 minus the number of other bytes in the packet

#define CMD_MAGIC_BYTE_VAL  0xAB    ///< Value of magic number in remote command header

// Command opcodes
#define SWITCH_LOAD_OPCODE  0x01


//-------------------------------------------------------------------------
// Command Structures
//------------------------------------------------------------------------

typedef struct _remote_cmd_header_t {
   uint8_t magic_num;   ///< magic word that indicates that this is a remote command. Always 0xAB
   uint8_t bodyLength;  ///< length of the body of the packet in bytes not including this header
   uint8_t opcode;      ///< Command opcode telling how to decode the body
} remote_cmd_header_t;

// protocol command structure. This is the generic structure of the commands to 
// be sent wirelessly. They can be many different types depending on the 'command'
// value. For example command=0x1 will switch a load on
typedef struct _remote_cmd_t {
    remote_cmd_header_t header;
    uint8_t body[MAX_BODY_SIZE];
} remote_cmd_t;


typedef struct _switch_load_cmd_t {
    remote_cmd_header_t header;
    uint8_t load_id;            ///< which load number to switch on
    uint8_t on_off;             ///< whether to switch the load on (1) or off (0)
} switch_load_cmd_t;


//-------------------------------------------------------------------------
// Load Switching
//------------------------------------------------------------------------

#define SWITCH_ON           (1)
#define SWITCH_OFF          (0)

// supported switch IDs
#define LOAD_ID_OUTLET_0    (0)

//-------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------
int send_load_switch_cmd(uint8_t load_id, uint8_t on_off);
int handle_received_command(uint8_t *cmd_buf, int cmd_length);
int switch_load(uint8_t load_id, uint8_t on_off);
int cmd_switch_load_handler(remote_cmd_t *cmdbuf);

#ifdef	__cplusplus
}
#endif

#endif	/* REMOTE_ACTIONS_H */

