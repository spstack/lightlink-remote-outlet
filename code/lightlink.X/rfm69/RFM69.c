/**
 * @file
 * @brief C Library for the RFM69 Module
 * @author Scott Stack 
 * 
 * @details This library is *heavily* inspired by the lowpowerlabs C++ arduino library
 * found here: https://github.com/LowPowerLab/RFM69
 */

#include <xc.h>
#include "../main.h"
#include "../timer.h"
#include "../config.h"
#include "../logging.h"
#include "../spi.h"
#include "RFM69.h"
#include "RFM69registers.h"

//-------------------------------------------------------------------------
// INTERNAL DATA
//------------------------------------------------------------------------
static uint8_t _currentMode;        // current operating mode only used in this file
static bool _ack_received;          // indicates that an acknowledgment has been received.
static uint8_t _cur_node_id;        // target node ID that this module listens for
static rfm69_buffer_t _rx_buffer;   // Buffer holding current RX packet


//-------------------------------------------------------------------------
// INTERNAL FUNCTIONS 
//------------------------------------------------------------------------
bool RFM69_receivePacketInternal(void);
void RFM69_resetRxBuf(void);
bool RFM69_canSend(void);

//-------------------------------------------------------------------------
// SYSTEM SPECIFIC FUNCTIONS - to be populated depending on platform
//------------------------------------------------------------------------

/**
 * @brief perform any output pin initialization, called during init of the module
*/
void RFM69_setup_pins(void)
{
    // Set reset pin to output
    RF_RESET_TRIS = 0;
    RFM69_set_reset_pin(0);
}

/**
 * @brief Set the chip select pin either high (inactive) or low (active)
*/
void RFM69_set_chipselect(uint8_t level)
{
    RF_CS_PIN = level;
}


/**
 * @brief Set the modules reset pin to specified level
*/
void RFM69_set_reset_pin(uint8_t level)
{
    RF_RESET_PIN = level;
}

/**
 * @brief Get current level of the RF interrupt pin (DIO0 on the module)
*/
uint8_t RFM69_get_rf_int_val(void)
{
    return RF_INT_PIN;
}

/**
 * @brief Function to pet watchdog within RFM library
*/
void RFM69_pet_watchdog(void)
{
    CLRWDT();
}


//-------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------

/**
 * @brief - Initialize the RFM69 Module
 * @details - This is a blocking function that will not return until the module is initialized successfully.
 * If something goes wrong it will never return and instead flash an error message on an LED
 * @param[in] freqBand - which frequency this module is using (only supports 915 at the moment)
 * @param[in] nodeID - the ID that this node will use to identify itself and listen for packets on
 * @param[in] networkID - the 8 bit network ID that this module will listen for messages on (cannot be zero!)
*/
void RFM69_init(uint8_t freqBand, uint16_t nodeID, uint8_t networkID)
{   
    uint8_t i;
    uint32_t initStartTime;
    
    // MAIN CONFIGURATION THAT IS SET ON BOOT
    // First entry is register number and second is register value
    const uint8_t CONFIG[][2] =
    {
      /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
      /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
      /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_100000}, // Set to 100kbps bitrate
      /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_100000},
      /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
      /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

      /* 0x07 */ { REG_FRFMSB, (uint8_t) RF_FRFMSB_915},
      /* 0x08 */ { REG_FRFMID, (uint8_t) RF_FRFMID_915},
      /* 0x09 */ { REG_FRFLSB, (uint8_t) RF_FRFLSB_915},

      // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
      // +17dBm and +20dBm are possible on RFM69HW
      // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
      // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
      // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
      /* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
      /* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

      // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
      /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
      //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
      /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using - this will be changed as the mode changes
      /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
      /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
      /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
      /* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE }, // default 3 preamble bytes 0xAAAAAA
      /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
      /* 0x2F */ { REG_SYNCVALUE1, 0xAA },
      /* 0x30 */ { REG_SYNCVALUE2, 0x55 },  // NETWORK ID - turned off because doesn't work in PIC compiler. Set later
      /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
      /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
      // /* 0x39 */ { REG_NODEADRS, nodeID }, // Will be written later in the function...
      /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
      /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
      //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
      /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
      {255, 0}
    };

    // Let module out of reset by setting reset pin as output then set to low
    RFM69_setup_pins();

    // Reset the module to ensure starting from known point (10ms wait is performed in this function)
    RFM69_reset();
    
    //Set CS pin high initially
    RFM69_set_chipselect(1);
    
    //implement timeout in case the RF module doesn't respond
    initStartTime = timer_get_ticks();
    
    //make sure that module is connected by updating the sync bytes and making
    //sure that the device responds
    do
    {
        if (timer_has_time_elapsed(initStartTime, RFM69_INIT_TIMEOUT_MS))
        {
            initStartTime = timer_get_ticks();
            log_error(CRITICAL_LEVEL, ERROR_RF_INIT);
            RFM69_reset();
        }

        // Pet watchdog while we do this
        RFM69_pet_watchdog();

        RFM69_writeReg(REG_SYNCVALUE1, 0xAA);
    } while (RFM69_readReg(REG_SYNCVALUE1) != 0xAA);

    initStartTime = timer_get_ticks();
    do
    {
        if (timer_has_time_elapsed(initStartTime, RFM69_INIT_TIMEOUT_MS))
        {
            initStartTime = timer_get_ticks();
            log_error(CRITICAL_LEVEL, ERROR_RF_INIT);
            RFM69_reset();
        }

        // Pet watchdog while init is happening
        RFM69_pet_watchdog();
        
        RFM69_writeReg(REG_SYNCVALUE2, 0x55);
    } while (RFM69_readReg(REG_SYNCVALUE2) != 0x55);
    
    
    // Set all default config options
    for (i = 0; CONFIG[i][0] != 255; i++)
    {
        RFM69_writeReg(CONFIG[i][0], CONFIG[i][1]);
    }    
    
    //Set network id
    RFM69_writeReg(REG_SYNCVALUE2, networkID);
    
    // write node ID
    RFM69_writeReg(REG_NODEADRS, nodeID);
    _cur_node_id = nodeID;   // this is what will be checked to see whether a packet is meant for this module
    
    //set the operating mode of the module to standby for now
    RFM69_setMode(RF69_MODE_STANDBY);
    
    _ack_received = false;
}


/**
 * @brief Reset the state of the internal RX buffer 
*/
void RFM69_resetRxBuf(void)
{
    _rx_buffer.has_data = false;
    _rx_buffer.length = 0;
    _rx_buffer.overrun = false;
}


/**
 * @brief - Read a single config register
*/
uint8_t RFM69_readReg(uint8_t addr)
{
    uint8_t val;
    int retval = SPI_NO_ERR;

    addr &= 0x7F; //clear bit 7 to indicate read
    
    // Write the address, then read the value
    RFM69_set_chipselect(0);
    spi_writebyte(addr);
    spi_readbyte(&val);
    RFM69_set_chipselect(1);
    
    return val;
}

/**
 * @brief - Write a single config register 
*/
void RFM69_writeReg(uint8_t addr, uint8_t val)
{
    addr |= 0x80; // set bit 7 indicates a write
    
    RFM69_set_chipselect(0);
    spi_writebyte(addr); 
    spi_writebyte(val);
    RFM69_set_chipselect(1);
}

/**
 * @brief - Get the current operating mode of the module
*/
uint8_t RFM69_getMode(void)
{
    return _currentMode;
}

/**
 * @brief - Set the operating mode of the RFM69 module
*/
void RFM69_setMode(uint8_t newMode)
{
    if (newMode == _currentMode)
    {
        return;
    }

    switch (newMode) {
        case RF69_MODE_TX:
            RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            //setHighPowerRegs(true);
            break;
        case RF69_MODE_RX:
            RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            //setHighPowerRegs(false);
            break;
        case RF69_MODE_SYNTH:
            RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69_MODE_STANDBY:
            RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69_MODE_SLEEP:
            RFM69_writeReg(REG_OPMODE, (RFM69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default:
            return;
    }
    
    _currentMode = newMode;
}


/**
 * @brief Checks receive signal strength against threshold to see whether it's safe to send
 * @details The RF module is single duplex, so it cannot send if something else is already transmitting
 * on the channel. This function helps to probe the channel to see whether something is already using it. 
 * We can avoid contention this way
*/
bool RFM69_canSend(void)
{
    int16_t wait_time_mask = 0x3; // limit range of rand nums to [0, 3]
    int16_t rssi_val = 0;
    uint32_t start_time = timer_get_ticks();
    uint8_t irq_reg = 0;

    // Make sure we're in RX mode to sense whether we can send
    if (RFM69_getMode() != RF69_MODE_RX)
    {
        RFM69_setMode(RF69_MODE_RX);
    }


    // Now check to make sure nothing is still transmitting, repeat until no traffic detected or max time
    do {
        // Reset Rx to reset the RSSI sensing
        RFM69_writeReg(REG_PACKETCONFIG2, RFM69_readReg(REG_PACKETCONFIG2) | RF_PACKET2_RXRESTART);

        // Wait for Rx to be ready before reading Rx
        while((RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0)
        {
            if (timer_has_time_elapsed(start_time, RF69_MAX_SEND_WAIT_MS))
            {
                return false;
            }
        }

        // Check for received packet to make sure one didn't come in since last time we checked.
        // If so, transfer to internal buffer
        RFM69_checkForRxData();
        
        // Wait a random small amount of time to let RSSI detect a signal from another transmitter
        timer_wait_ms(rand_lfsr16() & wait_time_mask);
        
        // Check to see whether the RSSI limit has exceeded the threshold (there's traffic on the link)
        rssi_val = (int16_t)RFM69_readReg(REG_RSSIVALUE);
        rssi_val = -rssi_val;
        rssi_val >>= 1; // divide by 2 to get value in dBm (per datasheet)
        if (rssi_val < CSMA_LIMIT)
        {
            // Clear to send!
            return true;
        }
    
    } while (!timer_has_time_elapsed(start_time, RF69_MAX_SEND_WAIT_MS));
    
    // timed out, return false
    return false;
}


/**
 * @brief - Send one data frame to specified address. 
 * @param[in] toAddress - node ID of the target device to send data to
 * @param[in] dataToSend - buffer to data to send
 * @param[in] dataLength - number of bytes in the data to send
 * @param[in] requestAck - will tell the receiver to send an acknowledgment packet upon receipt.
 * @param[in] sendAck - indicates that this frame is an acknowledgment packet sent in response to another received packet.
 * 
 * @return true on success or false on failure
 */
bool RFM69_sendFrame(uint8_t toAddress, uint8_t *dataToSend, uint8_t dataLength, bool requestAck, bool sendAck)
{
    uint8_t i, ctrlByte;
    uint32_t startTime;
    uint8_t startMode;
    
    //first disable the interrupt for RF INT pin because we're going to change it's usage
    // RF_INT_IOC = 0;
    
    //store current operating mode at start before changing to standby so it can be restored later
    startMode = RFM69_getMode();
    RFM69_setMode(RF69_MODE_STANDBY); //put device in standby while we fill buffer
    RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); //set DIO0 to mean 'send done'
    
    // check max length
    if (dataLength > RF69_MAX_DATA_LEN)
    {
        dataLength = RF69_MAX_DATA_LEN;
    }
    
    //set ctrl byte (user defined) to indicate if we want an ack for this packet
    if (requestAck)
    {
        ctrlByte = RFM69_CTL_REQACK;
    }
    else if (sendAck)
    {
        ctrlByte = RFM69_CTL_SENDACK;
    }
    else
    {
        ctrlByte = 0x0;
    }

    // Load packet header first
    RFM69_set_chipselect(0);
    spi_writebyte(REG_FIFO | RF69_REG_WRITE_MASK);  // set bit 7 to indicate a write to the FIFO
    spi_writebyte(dataLength + (RF69_NUM_PKT_HEADER_BYTES - 1));   // add 2 to length to include address field and crtlByte (which is only added by this driver)
    spi_writebyte(toAddress);
    spi_writebyte(ctrlByte);
    for (i = 0; i < dataLength; i++)
    {
        spi_writebyte(((uint8_t *)dataToSend)[i]);
    }
    RFM69_set_chipselect(1);
    
    //wait for sending to complete
    RFM69_setMode(RF69_MODE_TX);
    startTime = timer_get_ticks();
    while (RFM69_get_rf_int_val() == 0)
    {
        if (timer_has_time_elapsed(startTime, RFM69_SEND_TIMEOUT_MS))
        {
            log_error(ERROR_LEVEL, ERROR_RF_SEND_TIMEOUT);
            RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
            RFM69_setMode(startMode); //set mode back to what it was at the start
            return false;
        }
    }
    
    RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
    RFM69_setMode(startMode); //set mode back to what it was at the start
    // RF_INT_IOC = 1; //enable interrupt again 
    return true;
    
}

/**
 * @brief High level function to send a packet of data
 * @details Includes retries and guarantees receipt of data by requesting acknowledgments
 */
bool RFM69_sendPacket(uint8_t toAddress, uint8_t *dataToSend, uint8_t dataLength, bool requestAck)
{
    bool done = false;
    uint8_t retries = 0;
    uint32_t startTime;
    
    // To simplify things for now, only allow sending packets less than max frame size
    if (dataLength > RF69_MAX_DATA_LEN)
    {
        return false;
    }

    // Ensure we're clear to send a packet
    if (!RFM69_canSend())
    {
        // Couldn't determine if clear to send
        // attempting to send anyway...
        log_error(INFO_LEVEL, ERROR_RF_NOT_CTS);
    }
    
    _ack_received = false;
    
    // Retry transmission if it doesn't go through
    while (retries < RFM69_MAX_TX_RETRIES)
    {
        RFM69_sendFrame(toAddress, dataToSend, dataLength, requestAck, false);
        
        if (requestAck)
        {
            // Now enable rx mode and wait for ack or timeout
            RFM69_enableReceiveMode();
            startTime = timer_get_ticks();
            while (!timer_has_time_elapsed(startTime, RFM69_ACK_RX_TIMEOUT_MS))
            {
                RFM69_checkForRxData();
                
                if (_ack_received)
                {
                    _ack_received = false;
                    return true;
                }
            }
        }
        else
        {
            // if not requesting an ack, just return true here
            return true;
        }
        retries++;
    }
    
    log_error(DEBUG_LEVEL, ERROR_RF_NO_ACK);
    return false;
}

/**
 * @brief - Enable the module to receive packets. Must call this function to start receiving data
 */
void RFM69_enableReceiveMode(void)
{
    RFM69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
    RFM69_setMode(RF69_MODE_RX);
}


/**
 * @brief Check for received packet and transfer to internal buffer if packet is ready
*/
bool RFM69_receivePacketInternal(void)
{
    uint8_t packetLength;
    uint8_t ctrlByte;
    uint8_t targetId;
    uint8_t i;

    // Make sure payload is ready to be received
    if (RFM69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    {
        // If there's already a packet in the buffer, indicate overrun :(
        if (_rx_buffer.has_data) {
            _rx_buffer.overrun = true;
        }

        // Put device in standby while we fill buffer
        RFM69_setMode(RF69_MODE_STANDBY);
        
        // Read in the packet header
        RFM69_set_chipselect(0);
        spi_writebyte(REG_FIFO & 0x7F);  // clear bit 7 to indicate a read
        spi_readbyte(&packetLength);
        packetLength -= (RF69_NUM_PKT_HEADER_BYTES - 1); //subtract 2 to get length of payload not including control byte and address byte
        spi_readbyte(&targetId);
        spi_readbyte(&ctrlByte);

        // Sanity check packet length
        if (packetLength > RF69_MAX_DATA_LEN) {
            // set length to zero so rx_buffer isn't populated, but continue to clear the FIFO
            packetLength = 0;
            targetId = RFM69_INVALID_NODE_ADDR;
            log_error(ERROR_LEVEL, ERROR_RF_INVALID_PKT);
        }
        
        // Read the body of the packet.
        for (i = 0; i < packetLength; i++)
        {
            spi_readbyte(&_rx_buffer.data[i]);
        }
        RFM69_set_chipselect(1);

        // Check to make sure fifo is actually empty, if not clock out all remaining bytes
        while (RFM69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS2_FIFONOTEMPTY)
        {
            (void)RFM69_readReg(REG_FIFO);
            log_error(INFO_LEVEL, ERROR_RF_EXTRA_DATA);
        }
        
        // If this packet is addressed to us, handle the ack and return true
        if ((targetId == RFM69_getNodeId()) || (targetId == RF69_BROADCAST_ADDR))
        {
            // send an ack if sender requested it
            if (ctrlByte & RFM69_CTL_REQACK)
            {
                // send an ack packet to the sender
                RFM69_sendFrame(RFM69_getNodeId(), &ctrlByte, 0, false, true);
            }

            // If this is an ack packet, mark it and reset the buffer to indicate this isn't a real packet
            if (ctrlByte & RFM69_CTL_SENDACK)
            {
                _ack_received = true;
                RFM69_resetRxBuf();
            }
            else
            {
                // Indicate there's a packet ready
                _rx_buffer.length = packetLength;
                _rx_buffer.has_data = true;
            }
            
            
        }
        else
        {
            // Packet is not addressed to this node. Clear packet and return false
            _rx_buffer.length = 0;
            _rx_buffer.has_data = false;
        }

        RFM69_setMode(RF69_MODE_RX);
        return _rx_buffer.has_data;
    }
    
    return false;
}


/**
 * @brief Transfer a received packet from the modules internal buffer to the application
 * @details This is the main function that should be polled by the application on a periodic basis
 * it will check to see whether there are new packets to receive and transfer them if so.
 * 
 * @param[out] buf - buffer to place the received packet
 * @param[out] rcvd_pkt_size - size of the received packet in bytes
 * @param[in] buf_size - size of the input buffer in bytes (used to prevent overrun)
 * 
 * @returns true if packet has been received, false otherwise
*/
bool RFM69_receivePacket(uint8_t *buf, uint8_t *rcvd_pkt_size, uint16_t buf_size)
{
    uint8_t i;
    bool pkt_received = false;

    // Check for NULL pointers
    if ((rcvd_pkt_size == NULL) || (buf == NULL))
    {
        return false;
    }

    // check for data, and transfer it to caller if there is
    if (RFM69_checkForRxData())
    {
        // Ensure buffer is big enough
        if (buf_size >= _rx_buffer.length)
        {
            *rcvd_pkt_size = _rx_buffer.length;
            for (i = 0; i < _rx_buffer.length; i++)
            {
                buf[i] = _rx_buffer.data[i];
            }

            // Reset the buffer
            RFM69_resetRxBuf();
            return true;
        }
        else
        {
            // If specified buffer isn't big enough just clear everything and return false
            RFM69_resetRxBuf();
            return false;

        }
    }
        
    return false;
}

/**
 * @brief Poll to see whether there is any new data to receive. 
 * @details This is a non-blocking way to check whether there's data to be read from the module.
 * if it detects that the module has received a packet, it will also transfer that packet into
 * the libraries internal buffer
 * @return- true if there was data received or false if not
*/
bool RFM69_checkForRxData(void)
{
    bool have_packet = false;

    // If there's a packet already waiting, indicate there's data
    if (_rx_buffer.has_data)
    {
        have_packet = true;
    }

    // Check receive interrupt pin is set, then indicate that there's a packet waiting!
    if ((RFM69_getMode() == RF69_MODE_RX) && (RFM69_get_rf_int_val() == 1))
    {
        RFM69_receivePacketInternal();
        have_packet = true;
    }

    // Check for overrun
    if (_rx_buffer.overrun)
    {
        // Indicate error, but don't need to actually clear anything because newest packet has overwritten old one
        log_error(ERROR_LEVEL, ERROR_RF_RX_OVERFLOW);
    }
    
    return have_packet;
}

/**
 * @brief Reset the module by setting the reset pin high
*/
void RFM69_reset(void)
{
    RFM69_set_reset_pin(1);
    timer_wait_ms(1);
    RFM69_set_reset_pin(0);

    // Wait 10ms for device to come out of reset
    timer_wait_ms(10);
}


/**
 * @brief Return the current node ID that's being used
*/
uint8_t RFM69_getNodeId(void)
{
    return _cur_node_id;
}

/**
 * @brief Set the node ID that will be used by this module
*/
void RFM69_setNodeId(uint8_t nodeID)
{
    if (nodeID == RFM69_INVALID_NODE_ADDR)
    {
        return;
    }

    RFM69_writeReg(REG_NODEADRS, nodeID);
    _cur_node_id = nodeID;
}

/**
 * @brief Perform a quick register read to ensure the device is alive. Return true if so
 */
bool RFM69_isAlive(void)
{
    if (RFM69_readReg(REG_SYNCVALUE1) == 0xAA)
    {
        return true;
    }
    else
    {
        return false;
    }
}


