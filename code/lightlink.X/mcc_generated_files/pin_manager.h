/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.85.1
        Device            :  PIC16LF1455
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set BUTTON_INPUT aliases
#define BUTTON_INPUT_PORT               PORTAbits.RA0
#define BUTTON_INPUT_GetValue()           PORTAbits.RA0

// get/set nIRQ aliases
#define nIRQ_PORT               PORTAbits.RA1
#define nIRQ_GetValue()           PORTAbits.RA1

// get/set IO_RA4 aliases
#define IO_RA4_TRIS               TRISAbits.TRISA4
#define IO_RA4_LAT                LATAbits.LATA4
#define IO_RA4_PORT               PORTAbits.RA4
#define IO_RA4_WPU                WPUAbits.WPUA4
#define IO_RA4_ANS                ANSELAbits.ANSA4
#define IO_RA4_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define IO_RA4_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define IO_RA4_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define IO_RA4_GetValue()           PORTAbits.RA4
#define IO_RA4_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define IO_RA4_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define IO_RA4_SetPullup()      do { WPUAbits.WPUA4 = 1; } while(0)
#define IO_RA4_ResetPullup()    do { WPUAbits.WPUA4 = 0; } while(0)
#define IO_RA4_SetAnalogMode()  do { ANSELAbits.ANSA4 = 1; } while(0)
#define IO_RA4_SetDigitalMode() do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set LOAD_SWITCH aliases
#define LOAD_SWITCH_TRIS               TRISAbits.TRISA5
#define LOAD_SWITCH_LAT                LATAbits.LATA5
#define LOAD_SWITCH_PORT               PORTAbits.RA5
#define LOAD_SWITCH_WPU                WPUAbits.WPUA5
#define LOAD_SWITCH_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LOAD_SWITCH_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LOAD_SWITCH_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LOAD_SWITCH_GetValue()           PORTAbits.RA5
#define LOAD_SWITCH_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LOAD_SWITCH_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LOAD_SWITCH_SetPullup()      do { WPUAbits.WPUA5 = 1; } while(0)
#define LOAD_SWITCH_ResetPullup()    do { WPUAbits.WPUA5 = 0; } while(0)

// get/set SCK aliases
#define SCK_TRIS               TRISCbits.TRISC0
#define SCK_LAT                LATCbits.LATC0
#define SCK_PORT               PORTCbits.RC0
#define SCK_ANS                ANSELCbits.ANSC0
#define SCK_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define SCK_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define SCK_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define SCK_GetValue()           PORTCbits.RC0
#define SCK_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define SCK_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define SCK_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define SCK_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set SDI aliases
#define SDI_TRIS               TRISCbits.TRISC1
#define SDI_LAT                LATCbits.LATC1
#define SDI_PORT               PORTCbits.RC1
#define SDI_ANS                ANSELCbits.ANSC1
#define SDI_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define SDI_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define SDI_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define SDI_GetValue()           PORTCbits.RC1
#define SDI_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define SDI_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define SDI_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define SDI_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set SDO aliases
#define SDO_TRIS               TRISCbits.TRISC2
#define SDO_LAT                LATCbits.LATC2
#define SDO_PORT               PORTCbits.RC2
#define SDO_ANS                ANSELCbits.ANSC2
#define SDO_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define SDO_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define SDO_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define SDO_GetValue()           PORTCbits.RC2
#define SDO_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define SDO_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define SDO_SetAnalogMode()  do { ANSELCbits.ANSC2 = 1; } while(0)
#define SDO_SetDigitalMode() do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set DEBUG_LED aliases
#define DEBUG_LED_TRIS               TRISCbits.TRISC3
#define DEBUG_LED_LAT                LATCbits.LATC3
#define DEBUG_LED_PORT               PORTCbits.RC3
#define DEBUG_LED_ANS                ANSELCbits.ANSC3
#define DEBUG_LED_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define DEBUG_LED_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define DEBUG_LED_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define DEBUG_LED_GetValue()           PORTCbits.RC3
#define DEBUG_LED_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define DEBUG_LED_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define DEBUG_LED_SetAnalogMode()  do { ANSELCbits.ANSC3 = 1; } while(0)
#define DEBUG_LED_SetDigitalMode() do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set RC4 procedures
#define RC4_SetHigh()    do { LATCbits.LATC4 = 1; } while(0)
#define RC4_SetLow()   do { LATCbits.LATC4 = 0; } while(0)
#define RC4_Toggle()   do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define RC4_GetValue()         PORTCbits.RC4
#define RC4_SetDigitalInput()   do { TRISCbits.TRISC4 = 1; } while(0)
#define RC4_SetDigitalOutput()  do { TRISCbits.TRISC4 = 0; } while(0)

// get/set RX_LED aliases
#define RX_LED_TRIS               TRISCbits.TRISC5
#define RX_LED_LAT                LATCbits.LATC5
#define RX_LED_PORT               PORTCbits.RC5
#define RX_LED_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define RX_LED_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define RX_LED_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define RX_LED_GetValue()           PORTCbits.RC5
#define RX_LED_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define RX_LED_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCAF1 pin functionality
 * @Example
    IOCAF1_ISR();
 */
void IOCAF1_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCAF1 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCAF1 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF1_SetInterruptHandler(MyInterruptHandler);

*/
void IOCAF1_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCAF1 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCAF1_SetInterruptHandler() method.
    This handler is called every time the IOCAF1 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF1_SetInterruptHandler(IOCAF1_InterruptHandler);

*/
extern void (*IOCAF1_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCAF1 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCAF1_SetInterruptHandler() method.
    This handler is called every time the IOCAF1 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF1_SetInterruptHandler(IOCAF1_DefaultInterruptHandler);

*/
void IOCAF1_DefaultInterruptHandler(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/