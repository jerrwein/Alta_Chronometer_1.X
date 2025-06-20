/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

#include <xc.h>
#include "system.h"
#include "system_config.h"
#include "usb.h"
#include "leds.h"

extern uint8_t  counter_32[4];
// extern uint8_t  timer2_64[4];
extern TmData   tmr2_data;


/** CONFIGURATION Bits **********************************************/
// PIC16F1459 configuration bit settings:
#if defined (USE_INTERNAL_OSC)	    // Define this in system.h if using the HFINTOSC for USB operation
    // CONFIG1
    #pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
    #pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
    #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
    #pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
    #pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
    #pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
    #pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
    #pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
    #pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

    // CONFIG2
    #pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
    #pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (NO CPU system divide)
    #pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
    #pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
    #pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
    #pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
    #pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
    #pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
    #pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)
#else
    // CONFIG1
    #pragma config FOSC = HS        // Oscillator Selection Bits (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
    #pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
    #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
    #pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
    #pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
    #pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
    #pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
    #pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
    #pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

    // CONFIG2
    #pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
    #pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (NO CPU system divide)
    #pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
    #pragma config PLLMULT = 4x     // PLL Multipler Selection Bit (4x Output Frequency Selected)
    #pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
    #pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
    #pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
    #pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
    #pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)
#endif
/*********************************************************************
* Function: void SYSTEM_Initialize( SYSTEM_STATE state )
*
* Overview: Initializes the system.
*
* PreCondition: None
*
* Input:  SYSTEM_STATE - the state to initialize the system into
*
* Output: None
*
********************************************************************/

// Original
#if 1
void SYSTEM_Initialize( SYSTEM_STATE state )
{
    switch(state)
    {
        case SYSTEM_STATE_USB_START:
            #if defined(USE_INTERNAL_OSC)
                //Make sure to turn on active clock tuning for USB full speed 
                //operation from the INTOSC
                OSCCON = 0xFC;  //HFINTOSC @ 16MHz, 3X PLL, PLL enabled
                ACTCON = 0x90;  //Active clock tuning enabled for USB
            #endif
            // Green LED
            LED_Enable(LED_GREEN);
            LED_Off(LED_GREEN);
  
            // Blue LED
            LED_Enable(LED_BLUE);
            LED_Off(LED_BLUE);
            
            // Configure IR Detectors as I/O inputs
            TRISAbits.TRISA4 = 1;
            TRISAbits.TRISA5 = 1;
            ANSELAbits.ANSELA = 0;   /* Important */
 //         ANSELAbits.ANSA4 = 0;    /* Not working - Investigate? */

            // All PortA IOC positive & negative edge detectors off
            // IOCAPbits.IOCAP = 0;
            // IOCBNbits.IOCAN = 0;
            // A4 rising edge detector on
            // IOCAPbits.IOCAP4 = 1;
            // A5 falling edge detector on
            // IOCANbits.IOCAN7 = 1;
            break;
            
        case SYSTEM_STATE_USB_SUSPEND: 
            break;
            
        case SYSTEM_STATE_USB_RESUME:
            break;
    }
}
#endif

unsigned char led_toggle = 0;
unsigned int ioc_hits = 0;

void interrupt SYS_InterruptHigh(void)
{
    #if defined(USB_INTERRUPT)
        USBDeviceTasks();
    #endif

#ifdef JMW_TIMER1_INT
        // Handle Timer1 interrupt (e.g., if enabled)
    if (PIR1bits.TMR1IF)    // Check TMR1IF flag
    {
        // Reset interrupt flag
        PIR1bits.TMR1IF = 0;

        // Add your interrupt handler code here (e.g., toggle an LED)
//        LED_Toggle (LED_GREEN);

        if (++counter_32[3] == 0)
        {
            if (++counter_32[2] == 0)
            {
                if (++counter_32[1] == 0)
                {
                    counter_32[0]++;
                }
            }
        }
    }
#endif

#ifdef JMW_TIMER2_INT
        // Handle Timer2 interrupt (e.g., if enabled)
    if (PIR1bits.TMR2IF)    // Check TMR2IF flag
    {
        // Reset interrupt flag
        PIR1bits.TMR2IF = 0;

        // Add your interrupt handler code here (e.g., toggle an LED)
//        LED_Toggle (LED_GREEN);
#if 0
        if (++timer2_64[0] == 0)
        {
            if (++timer2_64[1] == 0)
            {
                if (++timer2_64[2] == 0)
                {
                    timer2_64[3]++;
                }
            }
        }
#endif

#if 0
        if (++tmr2_data.tm_u8[0] == 0)
        {
            if (++tmr2_data.tm_u8[1] == 0)
            {
                if (++tmr2_data.tm_u8[2] == 0)
                {
                    tmr2_data.tm_u8[3]++;
                }
            }
        }
#endif

#if 1
        tmr2_data.tm_u32++;
#endif
    }
#endif

#ifdef JMW_IOC_RB7_INT
    // Check if IOC interrupt flag is set
    if (INTCONbits.IOCIF)
    {
        ioc_hits++;
        if (IOCBFbits.IOCBF7)
        {
            // Clear the individual bit flag
            IOCBFbits.IOCBF7 = 0;
            // Clear the collective bits interrupt flag
            INTCONbits.IOCIF = 0;
            // Toggle an LED
            LED_Toggle(LED_D3);
        }
    }
#endif
}
