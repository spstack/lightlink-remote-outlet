/**
 * @brief Device configuration bits and configuration code for the PIC16F18146 microcontroller
 * @author Scott Stack
*/
#include <xc.h>
#include "config_16F18146.h"
#include "main.h"
#include "timer.h"
#include "uart.h"
#include "logging.h"

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_16MHz// Reset Oscillator Selection bits (HFINTOSC (16 MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config VDDAR = HI       // VDD Range Analog Calibration Selection bit (Internal analog systems are calibrated for operation between VDD = 2.3 - 5.5V)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up Timer Selection bits (PWRT is disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit is ignored)
#pragma config DACAUTOEN = OFF  // DAC Buffer Automatic Range Select Enable bit (DAC Buffer reference range is determined by the REFRNG bit)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection bit (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled; ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCKED One-Way Set Enable bit (The PPSLOCKED bit can be set and cleared as needed (unlocking sequence is required))
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a reset)
#pragma config DEBUG = OFF      // Background Debugger (Background Debugger disabled)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = ON        // WDT Operating Mode bits (WDT enabled regardless of Sleep; SEN bit is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = LFINTOSC// WDT Input Clock Select bits (WDT reference clock is the 31.0 kHz LFINTOSC)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash (SAF) Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block is NOT write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block is NOT write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register is NOT write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is NOT write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash (SAF) Write Protection bit (SAF is NOT write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored)

// CONFIG5
#pragma config CP = OFF         // Program Flash Memory Code Protection bit (Program Flash Memory code protection is disabled)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// Random number generator seed
static uint16_t _rand_seed;


/**
 * @brief initialize all peripherals
*/
void initialize_peripherals(void)
{
    init_oscillator();
    
    init_GPIOs();

    init_SPI();
    
    init_timers();

    init_UART();

    rand_lfsr_seed(rand_sample_adc());
}


/**
 * @brief initialize oscillator module(s)
*/
void init_oscillator(void)
{
    // Nothing to do for now
}

void enable_interrupts(void)
{
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
}

void disable_interrupts(void)
{
    INTCONbits.GIE = 0;
    INTCONbits.PEIE = 0;
}

void enable_pli_int(void)
{
    PLI_IOC_EN = 1;
    PIE0bits.IOCIE = 1; //enable interrupt on change module
}

void disable_pli_int(void)
{
    PLI_IOC_EN = 0;
    PIE0bits.IOCIE = 0;
}

/**
 * @brief Initialize all GPIO and peripheral pin mappings
 */
void init_GPIOs(void)
{
    // Turn off all analog functionality
    ANSELC = 0;
    ANSELB = 0;
    ANSELA = 0;
    
    // set LED to output and turn off
    DEBUG_LED_TRIS = 0;
    DEBUG_LED_PIN = 0;

    RF_RESET_TRIS = 1; // set rf module reset pin to input initially to make it high-Z as referenced in the datasheet

    RF_INT_TRIS = 1; //set RF int pin to input

    // Setup outlet control pin as output
    LOAD0_TRIS = 0;

    // Set channel select pins to input
    CHAN_SEL0_TRIS = 1;
    CHAN_SEL1_TRIS = 1;
    CHAN_SEL2_TRIS = 1;
    CHAN_SEL3_TRIS = 1;

    // Set button pin to input
    BUTTON_TRIS = 1;
    
    // Setup peripheral pin re-mapping for SPI
    SSP1CLKPPSbits.SSP1CLKPPS = PPS_PORTB | 4; // SPI CLK = RB4 (must match out)
    SSP1DATPPSbits.SSP1DATPPS = PPS_PORTB | 6; // MISO = RB6
    RB5PPSbits.RB5PPS = PPS_SDO1; // MOSI = RB5
    RB4PPSbits.RB4PPS = PPS_SCK1; // SPI CLK (out) = RB4

    // Setup peripheral pin re-mapping for UART output
    RC6PPSbits.RC6PPS = PPS_UART_TX1;

    RF_CS_TRIS = 0;  //set CS for RF module to output
    RF_CS_PIN = 1;   //set high by default (not active)
}

/**
 * @brief Initialize timer modules
 */
void init_timers(void)
{
    //setup timer 0 to be system clock
    T0CON0bits.OUTPS = 0; //select FOSC/4 as clock = 16MHz / 4 = 4MHz
    T0CON1bits.CKPS = 5; // prescalar of 32 = 125kHz
    T0CON1bits.CS = 2; // select FOSC/4 as clock source
    TMR0L = 0; // initialize to zero
    TMR0H = TIMER_INIT_VALUE_1MS; // this is the value that TMR0 will reset to when reached and generate an interrupt. This is 125 @ 125kHz or once every 1ms
    PIE0bits.TMR0IE = 1; //enable timer interrupt
    T0CON0bits.EN = 1; // enable the timer

    // Set system ticks (number of ms since reset) to zero
    SYSTEM_TICKS = 0;
}

/**
 * @brief Initialize SPI module (MSSP)
 */
void init_SPI(void)
{
    //set pin input/output modes
    SPI_MOSI_TRIS = 0;
    SPI_MISO_TRIS = 1;
    SPI_CLK_TRIS = 0;

    SSP1CON1bits.SSPM = 1; //SPI Master mode w/ 1MHz clock (FOSC / 16)
    SSP1CON1bits.SSPEN = 1; //enable the serial port
    SSP1STATbits.CKE = 1; // data changes in between clock edges

}


/**
 * @brief Initialize the UART module
*/
void init_UART(void)
{
    RC1STAbits.SPEN = 1;  // Enable serial port
    TX1STAbits.TXEN = 1; // Enable the transmitter
    SP1BRG = 25; // Configure 9600 baud
}



/**
 * @brief Get a true random number derived from lower bits of multiple ADC cycles
 * @note: not implemented at the moment.
*/
int16_t rand_sample_adc(void) {
    /* commented out for now
    int16_t return_val = 0;
    int8_t temp_val = 0;
    
    ADCON0bits.ADON = 1; // Turn ADC on
    
    for (int8_t i = 0; i < 8; i++) {
        ADCON0bits.CHS = 3; // Select AN3 channel (no significance)
        ADCON0bits.ADGO = 1; // Start conversion

        // Wait for conversion to complete
        while (ADCON0bits.ADGO == 1);

        // Done, sample lowest 2 bits
        temp_val = (ADRESL >> 6);
        return_val = (return_val << 2) | temp_val;
    }*/
    
    return 0xc1;
}

/**
 * @brief Generate a random 16 bit integer using an LFSR
*/
int16_t rand_lfsr16(void) {
    const uint16_t poly = 0xB400u;

    uint8_t lsb = _rand_seed & 1u;
    _rand_seed >>= 1;
    if (lsb) {
        _rand_seed ^= poly; 
    }

    return _rand_seed;
}

/**
 * Seed the randomizer
*/
void rand_lfsr_seed(uint16_t seed) {
    if (seed == 0) {
        log_error(ERROR_LEVEL, ERROR_ZERO_SEED);
        seed = 0xe3; // just choose random number
    }
    _rand_seed = seed;
}

/**
 * @brief Enter a critical section where interrupts are disabled
 */
void enter_critical_section(void)
{
    disable_interrupts();
}

/**
 * @brief Enter a critical section where interrupts are disabled
 */
void exit_critical_section(void)
{
    enable_interrupts();
}

