/* 
 * File:   config.h
 * Author: Scott
 *
 * Created on July 13, 2019, 12:59 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>

//========================PIN ASSIGNMENTS ==========================
#define DEBUG_LED_TRIS TRISAbits.TRISA5
#define DEBUG_LED_PIN LATAbits.LATA5

#define RF_CS_TRIS TRISCbits.TRISC3
#define RF_CS_PIN LATCbits.LATC3

#define RF_RESET_TRIS TRISCbits.TRISC4
#define RF_RESET_PIN LATCbits.LATC4

#define RF_INT_TRIS TRISAbits.TRISA4
#define RF_INT_PIN PORTAbits.RA4
// #define RF_INT_IOC IOCAPbits.IOCAP4

#define SPI_MOSI_TRIS TRISCbits.TRISC2
#define SPI_MISO_TRIS TRISCbits.TRISC1
#define SPI_CLK_TRIS TRISCbits.TRISC0

#define BUTTON_SENDMSG_PIN PORTAbits.RA0 // Note: there is no tris for this pin since it's always an input

#define LOAD0_TRIS TRISCbits.TRISC5
#define LOAD0_PIN LATCbits.LATC5
    
#define PLI_PIN PORTAbits.RA1
#define PLI_IOC_EN IOCAPbits.IOCAP1 // enable only positive edge for trigger
#define PLI_INT_FLAG IOCAFbits.IOCAF1

    
//========================== DEFINITIONS ===============================
#define TIMER_INIT_VALUE_1MS 131 //value that timer reg needs to be set to to interrupt every 1ms




void initialize_peripherals(void);
void init_oscillator(void);
void enable_interrupts(void);
void disable_interrupts(void);
void enable_pli_int(void);
void init_GPIOs(void);
void init_SPI(void);
void init_timers(void);

void LED_blinkErrorCode(uint8_t pattern);
void LED_resetIndication(void);

// Random number generation 
int16_t rand_sample_adc(void);
int32_t rand_lfsr_seed(uint32_t seed);
int16_t rand_lfsr16(void);


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

