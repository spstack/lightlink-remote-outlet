/* 
 * File:   RFM12B.h
 * Author: Scott
 *
 * Created on February 25, 2018, 11:26 AM
 */

#ifndef RFM12B_H
#define	RFM12B_H



#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

        
// ==== DEFINES ====

#define RF69_315MHZ             31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ             43
#define RF69_868MHZ             86
#define RF69_915MHZ             91

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MAX_SEND_WAIT_MS   50  // max time in ms to wait for RSSI to be below CSMA_LIMIT

// Register defines
#define RF69_REG_WRITE_MASK     0x80
#define RF69_REG_READ_MASK      0x7F

// Packet defines
#define RF69_NUM_PKT_HEADER_BYTES 3 // Number bytes that are included in the RFM69 packet header. For now, this is <length> <target_node_id> <control byte>

// Timeouts
#define RFM69_SEND_TIMEOUT_MS   50 // max time to wait for send complete interrupt (max packet size 66B / bitrate (4.8kbps) = 110ms plus some buffer)
#define RFM69_ACK_RX_TIMEOUT_MS 50 //max time to wait for an ack
#define RFM69_MAX_TX_RETRIES    3
#define RFM69_INIT_TIMEOUT_MS   500 // number of milliseconds to wait for response from module before declaring failure during initialization

// Network IDs
#define RFM69_DEFAULT_NETWORK_ID    (0x55)  ///< Default network ID to be used by remote
#define RF69_BROADCAST_ADDR         0
#define RFM69_INVALID_NODE_ADDR     0xFE    ///< Invalid target/node ID used internally

// Packet defines
#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)

// Modes of operation
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX                4 // TX MODE

    
// define CTLbyte bits 
#define RFM69_CTL_SENDACK           0x80
#define RFM69_CTL_REQACK            0x40


// ==== TYPES ====

/**
 * @brief Internal buffer used to hold RX data. Only holds one packet
*/
typedef struct _rfm69_buffer_t {
    uint8_t data[RF69_MAX_DATA_LEN];    ///< actual packet data
    uint8_t length;                     ///< length of the packet in 'data' in bytes
    bool has_data;                      ///< indicates whether the buffer has any data in it
    bool overrun;                       ///< indicates whether the rx buffer has been overrun and a packet has been dropped. This is just a notification, the buffer will still contain the most recent packet
} rfm69_buffer_t;



// ==== FUNCTIONS ====

// Functions that should be populated depending on system
void RFM69_setup_pins(void);
void RFM69_set_chipselect(uint8_t level);
void RFM69_set_reset_pin(uint8_t level);
void RFM69_pet_watchdog(void);

// Public API functions
void RFM69_init(uint8_t freqBand, uint16_t nodeID, uint8_t networkID);
uint8_t RFM69_readReg(uint8_t addr);
void RFM69_writeReg(uint8_t addr, uint8_t val);
uint8_t RFM69_getMode(void);
void RFM69_setMode(uint8_t newMode);
bool RFM69_sendFrame(uint8_t toAddress, uint8_t *dataToSend, uint8_t dataLength, bool requestAck, bool sendAck);
bool RFM69_sendPacket(uint8_t toAddress, uint8_t *dataToSend, uint8_t dataLength, bool requestAck);
bool RFM69_checkForRxData(void);
void RFM69_enableReceiveMode(void);
bool RFM69_receivePacket(uint8_t *buf, uint8_t *rcvd_pkt_size, uint16_t buf_size);
void RFM69_reset(void);
uint8_t RFM69_getNodeId(void);
void RFM69_setNodeId(uint8_t nodeID);
bool RFM69_isAlive(void);


#ifdef	__cplusplus
}
#endif

#endif	/* RFM12B_H */

