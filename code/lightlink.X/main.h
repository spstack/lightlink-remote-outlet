/**
 * @file
 * @brief Main C file for RF controlled outlet
 * @author Scott Stack 
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>


//-------------------------------------------------------------------------
// Build Configuration
//------------------------------------------------------------------------
// #define DEBUG_UART_OUT
#define DEBUG_LED_BLINK
    
    
//-------------------------------------------------------------------------
// Global Data
//------------------------------------------------------------------------

/// @brief Structure that contains all global data
typedef struct _global_data_t {
    // RF Module data
    uint8_t rf_receive_buf[64];         ///< receive buffer used for incoming messages
    uint8_t current_channel;            ///< Current channel setting of this module. Read from input DIP switches on front
    // bool rf_module_has_data = false; ///< Flag indicating that RF module has data

    // Periodic update status data
    uint32_t time_last_pwron_received;  ///< Time that last periodic poweron command was received 
    uint32_t time_last_pwron_sent;      ///< Time that last periodic poweron command was sent
    uint32_t next_wakeup_time;          ///< Amount of time in ms to wait until next periodic update message is sent (changes randomly each cycle)
    bool poweron_msg_received;          ///< Flag indicating whether this module has ever received a power on message

    // Current state
    uint8_t current_switch_val;         ///< Current state of the attached switch

    // Power loss
    bool pli_detected;                  ///< Power loss detected
} global_data_t;

extern global_data_t global_data;
extern uint32_t SYSTEM_TICKS;

//-------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------

// Logic level defines for clarity
#define HIGH    (1)
#define LOW     (0)

// Configuration
#define NUM_INITIAL_RETRIES     (3)     ///< Number of times to send an initial 
#define WAKEUP_POLL_PERIOD_MS   (2000)  ///< Average number of ms to wait before sending a power on message to other endpoint. Some random jitter will be added to this to reduce chance of message collision
                                        /// The min value here is the time it takes to send a message (~5ms)
#define INIT_TIME_WAIT_MS       (50)    ///< Number of ms to wait after poweron before proceeding w/ initialization. This is to ensure power is stable          


//-------------------------------------------------------------------------
// Error Levels
//------------------------------------------------------------------------

typedef enum _error_level_e {
    DEBUG_LEVEL = 0,
    INFO_LEVEL,
    ERROR_LEVEL,
    CRITICAL_LEVEL,
    NO_LOGGING,         // Always the last entry
} error_level_e;

#define DEFAULT_LOG_LEVEL       (INFO_LEVEL)    ///< The default log level that will be used
#define TRAP_ERROR              (0)             ///< Set to 1 to hang on first instance of an error and repeatedly flash error code (debug only)

//-------------------------------------------------------------------------
// ERROR CODES
//------------------------------------------------------------------------

#define NO_ERROR                0
#define ERROR_NULL_PTR          0x01    ///< NULL pointer passed to function
#define ERROR_INV_BUF_SIZE      0x02    ///< Invalid buffer size detected
// RFM error codes
#define ERROR_RF_SEND           0x10    ///< Problem sending data through RFM69 module
#define ERROR_RF_INIT           0x11    ///< Failure to initialize the RF module
#define ERROR_RF_NO_ACK         0x12    ///< Couldn't get ACK from remote module through RFM69
#define ERROR_RF_RX_OVERFLOW    0x13    ///< We received a packet from remote before the existing packet could be read from internal buffer
#define ERROR_RF_SEND_TIMEOUT   0x14    ///< Did not receive a transmit done interrupt after sending packet within allowed time frame
#define ERROR_RF_NOT_CTS        0x15    ///< When trying to send a packet, timed out waiting for it to be clear to send
#define ERROR_RF_NOT_RESP       0x16    ///< RF module is not responding to simple register read
#define ERROR_RF_INVALID_PKT    0x17    ///< Received a packet with an invalid size
#define ERROR_RF_EXTRA_DATA     0x18    ///< There was extra data left in the FIFO after reading a full packet
// remote cmd error codes
#define ERROR_CMD_MAGIC_NUM     0x20    ///< Wrong magic number detected in received command
#define ERROR_INVALID_OPCODE    0x21    ///< Invalid command opcode
// load switching
#define ERROR_INVALID_LOAD_ID   0x30    ///< invalid load_id was specified in switch command
// random num generator 
#define ERROR_ZERO_SEED         0x40    ///< Tried to seed the randomizer with a zero value



//-------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------
uint8_t read_channel_code_input(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

