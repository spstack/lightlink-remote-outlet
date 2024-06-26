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

#ifndef __SPI_DRIVER_H
#define __SPI_DRIVER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "spi_types.h"

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



#define INLINE  inline 

// -----------------------------------
// FUNCTIONS
// -----------------------------------

/* arbitration interface */
INLINE void spi_close(void);

bool spi_master_open(spi_modes spiUniqueConfiguration);
bool spi_slave_open(spi_modes spiUniqueConfiguration);
/* SPI native data exchange function */
uint8_t spi_exchangeByte(uint8_t b);
/* SPI Block move functions }(future DMA support will be here) */
void spi_exchangeBlock(void *block, size_t blockSize);
void spi_writeBlock(void *block, size_t blockSize);
void spi_readBlock(void *block, size_t blockSize);

int spi_readbyte(uint8_t *rx_byte);
int spi_writebyte(uint8_t b);

#endif // __SPI_DRIVER_H
