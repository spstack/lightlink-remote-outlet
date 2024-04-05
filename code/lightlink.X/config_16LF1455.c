// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV3 // CPU System Clock Selection Bit (CPU system clock divided by 3)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = DISABLED // PLL Enable Bit (3x or 4x PLL Disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic16lf1455.h>
#include "main.h"
#include "config.h"
#include "timer.h"


void initialize_peripherals(void)
{
    init_oscillator();
    
    init_GPIOs();
    
    init_SPI();
    
    init_timers();

    rand_lfsr_seed(rand_sample_adc());
}

void init_oscillator(void)
{
    uint32_t i;

    //choose 16MHz internal Oscillator source
    OSCCONbits.IRCF = 0xF;

    //wait for a little while for clock to stabilize before continuing
    for (i=0; i < 10000; i++);
    
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
    INTCONbits.IOCIE = 1; //enable interrupt on change module
}

void init_GPIOs(void)
{
    // Turn off all analog functionality
    ANSELC = 0;
    ANSELA = 0;
    
    DEBUG_LED_TRIS = 0; //set LED to output
    DEBUG_LED_PIN = 0;
    
    RF_CS_TRIS = 0;  //set CS for RF module to output
    RF_CS_PIN = 1;   //set high by default (not active)
    
    RF_RESET_TRIS = 1; // set rf module reset pin to input initially to make it high-Z as referenced in the datasheet

    RF_INT_TRIS = 1; //set RF int pin to input
    //RF_INT_IOC = 1; //set to interrupt on rising edge of RFM69 interrupt pin

    // Setup outlet control pin as output
    LOAD0_TRIS = 0;

    // BUTTON_SENDMSG_TRIS = 1; // Set button pin to input (doesn't exist... always an input)
}


void init_timers(void)
{
    //setup timer 0 to be system clock
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0; //select FOSC/4 as clock = 16MHz / 4 = 4MHz
    OPTION_REGbits.PS = 4; //prescalar of 32 = 125kHz
    TMR0 = TIMER_INIT_VALUE_1MS; //initialize TMR0 so that it overflows after 125 ticks @ 125kHz or 1ms
    TMR0IE = 1; //enable timer interrupt
    
    SYSTEM_TICKS = 0;
    
}

void init_SPI(void)
{
    SSPCON1bits.SSPM = 1; //SPI Master mode w/ 1MHz clock (FOSC / 16)
    SSPCON1bits.SSPEN = 1; //enable the serial port
    SSPSTATbits.CKE = 1; // data changes in between clock edges
    
    
    //set pin input/output modes
    SPI_MOSI_TRIS = 0;
    SPI_MISO_TRIS = 1;
    SPI_CLK_TRIS = 0;
    
}


//Code to blink our one LED with an error code specified by 'pattern'
//Each bit in pattern represents a short or long LED pulse.
// Short pulse = 0
// Long pulse = 1
void log_error(uint8_t pattern)
{
    uint8_t i, current_bit;

    CLRWDT();

    //wait 1 second off to indicate start of pattern
    DEBUG_LED_PIN = 0;
    timer_wait_ms(1000);

    CLRWDT();
    
    // for each bit in 'pattern' blink a long or short pulse
    for(i = 0; i < 8; i++)
    {
        current_bit = (pattern >> i) & 0x1;
        
        // if '1' then long pulse
        if (current_bit)
        {
            DEBUG_LED_PIN = 1;
            timer_wait_ms(300);
            DEBUG_LED_PIN = 0;
            timer_wait_ms(300);
        }
        else
        {
            DEBUG_LED_PIN = 1;
            timer_wait_ms(100);
            DEBUG_LED_PIN = 0;
            timer_wait_ms(500);   
        }

        CLRWDT();
    }

    
    DEBUG_LED_PIN = 0;
    timer_wait_ms(1000);
    DEBUG_LED_PIN = 1;
}

/**
 * @brief - blink LEDs in a specific pattern to indicate reset
*/
void LED_resetIndication(void)
{
    // Flash LED to indicate reset
    DEBUG_LED_PIN = 1;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 0;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 1;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 0;
    timer_wait_ms(250);
    DEBUG_LED_PIN = 1;
    
}



static uint16_t _rand_seed;

/**
 * @brief Get a true random number derived from lower bits of multiple ADC cycles
*/
int16_t rand_sample_adc(void) {
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
    }
    
    return return_val;
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
int32_t rand_lfsr_seed(uint32_t seed) {
    if (seed == 0) {
        log_error(ERROR_ZERO_SEED);
        seed = 0xe3; // just choose random number
    }
    _rand_seed = seed;
}

