/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mcc.h"
#include "spi_driver.h"

#pragma warning disable 520        

inline void spi_close(void)
{
    SSP1CON1bits.SSPEN = 0;
}

//con1 == SSPxCON1, stat == SSPxSTAT, add == SSPxADD
typedef struct { uint8_t con1; uint8_t stat; uint8_t add; } spi_configuration_t;
static const spi_configuration_t spi_configuration[] = {   
    {0x0, 0x40, 0x1 }
};

//Setup SPI for Master Mode
bool spi_master_open(spi_modes spiUniqueConfiguration)
{
    if(!SSP1CON1bits.SSPEN)
    {
        SSP1STAT = spi_configuration[spiUniqueConfiguration].stat;
        SSP1CON1 |= spi_configuration[spiUniqueConfiguration].con1 | 0x2A;
        SSP1CON2 = 0x00;
        SSP1ADD = spi_configuration[spiUniqueConfiguration].add | 0;

        SCK_SetDigitalOutput();
        return true;
    }
    return false;
}

//Setup SPI for Slave Mode
bool spi_slave_open(spi_modes spiUniqueConfiguration)
{
    if(!SSP1CON1bits.SSPEN)
    {
        SSP1STAT = spi_configuration[spiUniqueConfiguration].stat;
        SSP1CON1 |= spi_configuration[spiUniqueConfiguration].con1 | 0x25;
        SSP1CON2 = 0x00;
        SSP1ADD = spi_configuration[spiUniqueConfiguration].add | 0;
        
        SCK_SetDigitalInput();
        return true;
    }
    return false;
}


/**
 * Full duplex SPI function to either initiate a 
*/
uint8_t spi_exchangeByte(uint8_t b)
{
    SSP1BUF = b;
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    return SSP1BUF;
}

void spi_exchangeBlock(void *block, size_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        *data = spi_exchangeByte(*data );
        data++;
    }
}

// Half Duplex SPI Functions
void spi_writeBlock(void *block, size_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        spi_exchangeByte(*data++);
    }
}

void spi_readBlock(void *block, size_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        *data++ = spi_exchangeByte(0);
    }
}


/**
 * Blocking function to read a single byte from the SPI interface
*/
int spi_readbyte(uint8_t *rx_byte)
{
    uint32_t start_ticks = 0;

    // First write a zero to the buffer register to initiate a full byte of clock
    SSP1BUF = 0;

    // Wait for the status register to indicate transfer is complete
    start_ticks = timer_get_ticks();    
    while(!PIR1bits.SSP1IF) {
        if (timer_has_time_elapsed(start_ticks, SPI_READ_TIMEOUT_MS)) {
            return SPI_RX_TIMEOUT_ERR;
        }
    }
    PIR1bits.SSP1IF = 0;

    // Return the byte that was clocked in
    *rx_byte = SSP1BUF;

    return SPI_NO_ERR;
}

/**
 * Blocking function to write a single byte
*/
int spi_writebyte(uint8_t b)
{
    uint32_t start_ticks = 0;

    // write byte to the buffer register to initiate the transfer
    SSP1BUF = b;

    // Wait for byte to indicate complete
    start_ticks = timer_get_ticks();    
    while(!PIR1bits.SSP1IF) {
        if (timer_has_time_elapsed(start_ticks, SPI_WRITE_TIMEOUT_MS)) {
            return SPI_TX_TIMEOUT_ERR;
        }
    }
    PIR1bits.SSP1IF = 0;

    return SPI_NO_ERR;
}