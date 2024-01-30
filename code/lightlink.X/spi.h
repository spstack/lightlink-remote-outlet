/* 
 * File:   spi.h
 * Author: Scott
 *
 * Created on July 13, 2019, 10:18 PM
 */

#ifndef SPI_H
#define	SPI_H


// -----------------------------------
// Defines
// -----------------------------------

// Error codes
#define SPI_NO_ERR              (0)
#define SPI_RX_TIMEOUT_ERR      (-1)
#define SPI_TX_TIMEOUT_ERR      (-2)

// Timeout thresholds
#define SPI_READ_TIMEOUT_MS     (5)
#define SPI_WRITE_TIMEOUT_MS    (5)



#ifdef	__cplusplus
extern "C" {
#endif

uint8_t spi_exchangeByte(uint8_t b);
void spi_exchangeBlock(void *block, uint16_t blockSize);
void spi_writeBlock(void *block, uint16_t blockSize);
void spi_readBlock(void *block, uint16_t blockSize);

int spi_readbyte(uint8_t *rx_byte);
int spi_writebyte(uint8_t b);


#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */






