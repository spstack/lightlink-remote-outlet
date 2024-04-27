/**
 * @brief Configuration header for the PIC16F18146 microcontroller
 * @author Scott Stack
*/
#ifndef CONFIG_18146_H
#define CONFIG_18146_H

#include <xc.h>


//========================PIN ASSIGNMENTS ==========================
#define DEBUG_LED_TRIS TRISAbits.TRISA2
#define DEBUG_LED_PIN LATAbits.LATA2

#define RF_CS_TRIS TRISBbits.TRISB7
#define RF_CS_PIN LATBbits.LATB7

#define RF_RESET_TRIS TRISCbits.TRISC5
#define RF_RESET_PIN LATCbits.LATC5

#define RF_INT_TRIS TRISAbits.TRISA5
#define RF_INT_PIN PORTAbits.RA5
// #define RF_INT_IOC IOCAPbits.IOCAP5

#define SPI_MOSI_TRIS TRISBbits.TRISB5
#define SPI_MISO_TRIS TRISBbits.TRISB6
#define SPI_CLK_TRIS TRISBbits.TRISB4

#define BUTTON_TRIS TRISCbits.TRISC7
#define BUTTON_PIN PORTCbits.RC7

#define LOAD0_TRIS TRISCbits.TRISC4
#define LOAD0_PIN LATCbits.LATC4
    
#define PLI_PIN PORTAbits.RA4
#define PLI_IOC_EN IOCAPbits.IOCAP4 // enable only positive edge for trigger
#define PLI_INT_FLAG IOCAFbits.IOCAF4

#define CHAN_SEL0_TRIS TRISCbits.TRISC0
#define CHAN_SEL0_PIN PORTCbits.RC0
#define CHAN_SEL1_TRIS TRISCbits.TRISC1
#define CHAN_SEL1_PIN PORTCbits.RC1
#define CHAN_SEL2_TRIS TRISCbits.TRISC2
#define CHAN_SEL2_PIN PORTCbits.RC2
#define CHAN_SEL3_TRIS TRISCbits.TRISC3
#define CHAN_SEL3_PIN PORTCbits.RC3

    
//========================== DEFINITIONS ===============================
#define TIMER_INIT_VALUE_1MS 131 //value that timer reg needs to be set to to interrupt every 1ms

// PPS Defines for inputs
#define PPS_PORTA       (0x00)
#define PPS_PORTB       (0x08)
#define PPS_PORTC       (0x10)

#define PPS_SDO1        (0x1C) // Serial data out ID for PPS output mapping
#define PPS_SCK1        (0x1B) // serial clock out ID for PPS output
#define PPS_UART_TX1    (0x13) // UART1 Transmit


void initialize_peripherals(void);
void init_oscillator(void);
void enable_interrupts(void);
void disable_interrupts(void);
void enable_pli_int(void);
void init_GPIOs(void);
void init_SPI(void);
void init_UART(void);
void init_timers(void);

// Random number generation 
int16_t rand_sample_adc(void);
void rand_lfsr_seed(uint16_t seed);
int16_t rand_lfsr16(void);







#endif // CONFIG_18146_H
 