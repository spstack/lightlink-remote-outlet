/**
 * @brief Implements a SPI device driver
 */
#include <xc.h>
#include "main.h"
#include "config.h"
#include "spi.h"
#include "timer.h"


// Full Duplex SPI Functions
uint8_t spi_exchangeByte(uint8_t b)
{   
    uint8_t temp;
    
    //if write collision has occurred, clear the error (ignore for now)
    if (SSP1CON1bits.WCOL)
    {
        SSP1CON1bits.WCOL = 0;
    }
    
    //load byte into SPI write register to initiate transfer. Wait until done
    SSP1BUF = b;
    while(!SSP1IF);
    SSP1IF = 0;
    temp = SSP1BUF;
    
    return temp;
}

void spi_exchangeBlock(void *block, uint16_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        *data = spi_exchangeByte(*data );
        data++;
    }
}

// Half Duplex SPI Functions
void spi_writeBlock(void *block, uint16_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        spi_exchangeByte(*data++);
    }
}

void spi_readBlock(void *block, uint16_t blockSize)
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
    
    // Wait for spi to be done transferring
    while(!SSP1STATbits.BF);

    // Clear int flag just in case
    SSP1IF = 0;

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
    
    // Wait for spi to be done transferring
    while(!SSP1STATbits.BF);

    // Clear int flag just in case
    SSP1IF = 0;

    return SPI_NO_ERR;
}
