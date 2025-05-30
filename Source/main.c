
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

/** INCLUDES *******************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <xc.h>

#include "system.h"
#include <system_config.h>

#include "leds.h"

#include "usb.h"
#include "usb_device_hid.h"

#include "app_device_custom_hid.h"
#include "app_led_usb_status.h"

const unsigned char fw_revision[2] = {3, 14};
uint8_t counter_32[4] = {0x00, 0x00, 0x00, 0x00};
TmData tmr2_data;

enum led_state
{
    ON_1,
    OFF_1,
    ON_2,
    OFF_2
};

MAIN_RETURN main(void)
{
    int delay_cnt;
//    unsigned int loop_cnt;
    bool led_off = true;
    enum led_state ledState = ON_1;
    volatile uint16_t last_tmr2_lsw;
    volatile uint16_t test_tmr2_lsw;
    uint16_t led_cycle_start;
    
#ifdef JMW_TIMER1_INT
    // Configure Timer1        Fosc = 16 Mhz * PLL / CPU_DIV = 16M * 3 / 1 = 48 Mhz
    T1CONbits.TMR1ON = 0;     // Disable Timer1 initially
    T1CONbits.TMR1CS = 0;     // Select instruction clock source (Fosc/4 = 48/4 =  12 MHz)
//  T1CONbits.T1CKPS = 3;   // Prescaling (1:8)  500K/65536 = 22.89 Hz
//  T1CONbits.T1CKPS = 2;   // Prescaling (1:4)    3M/65536 = 45.77 Hz
    T1CONbits.T1CKPS = 1;   // Prescaling (1:2)    6M/65536 = 91.55 Hz
//    T1CONbits.T1CKPS = 0;   // No prescaling (1:1)  12M/65536 = 183.1 Hz
    T1GCONbits.TMR1GE = 0;  // No Gate Enable

    // Clear TMR1 registers
    TMR1H = 0;
    TMR1L = 0;
 //    TMR1 = 0; // clear timer value
    // JMW Timer 1 End
#endif // #ifdef JMW_TIMER1_INT

#ifdef JMW_TIMER2_INT
    // Configure Timer2          Fosc = 16 Mhz * PLL / CPU_DIV = 16M * 3 / 1 = 48 MHz
    // Pre-scaler input clock    Fosc/4 = 12 MHz
    T2CONbits.TMR2ON = 0;     // Disable Timer2 initially
//  T2CONbits.T2CKPS = 1;     // Select clock pre-scale of 4 (12MHz/4 =  3 MHz)
    T2CONbits.T2CKPS = 2;     // Select clock pre-scale of 16 (12MHz/16 =  750 KHz)
//  T2CONbits.T2CKPS = 0;      // Prescaling 1 = 12 MHz
//  T2CONbits.T2CKPS = 2;      // Prescaling 16 = 750 KHz
//  T2CONbits.T2CKPS = 3;      // Prescaling 64 = 187500 KHz
    T2CONbits.T2OUTPS = 0;     // Output post-scaling = 1

    // Timer2 Period Register (Compare))
//  PR2 = 149;                  // Divide by 150 (3M / 150 = 20 KHz, T = 50 us)
    PR2 = 149;                  // Divide by 150 (750K / 150 = 5 KHz, T = 200 us)

    // Clear TMR2 register
    TMR2 = 0;
#endif // #ifdef JMW_TIMER1_INT

#ifdef JMW_IOC_RB7_INT
    // Configure the IOC pin as an input
    TRISBbits.TRISB7 = 1;
    // All PortB IOC positive & negative edge detectors off
    IOCBPbits.IOCBP = 0;
    IOCBNbits.IOCBN = 0;
    // B7 rising edge detector on
//   IOCBPbits.IOCBP7 = 1;
    // B7 falling edge detector on
    IOCBNbits.IOCBN7 = 1;
    // Enable IOC interrupts
//    INTCONbits.IOCIE = 1;
#endif // #ifdef JMW_IOC_RB5_INT

    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();

#ifdef JMW_TIMER1_INT
    // Enable Timer1
    T1CONbits.TMR1ON = 1;
    // Enable interrupts from Timer1
    PIE1bits.TMR1IE = 1;    // Enable Timer1 interrupt
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
#endif

#ifdef JMW_TIMER2_INT
    // Enable Timer2
    T2CONbits.TMR2ON = 1;
    // Enable interrupts from Timer2
    PIE1bits.TMR2IE = 1;    // Enable Timer2 interrupt
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
#endif

#ifdef JMW_IOC_RB7_INT
    // Enable IOC interrupts
    INTCONbits.IOCIE = 1;
#endif

    tmr2_data.tm_u32 = 0x00000000;

#if defined(JMW_TIMER1_INT) || defined(JMW_TIMER2_INT) || defined(JMW_IOC_RB7_INT)
    // JMW - Enable global interrupts
    INTCONbits.GIE = 1;     // Enable global interrupts
#endif

//  loop_cnt = 0;
    led_cycle_start = tmr2_data.tm_16[0];
    LED_On (LED_GREEN);

    while(1)
    {
//        SYSTEM_Tasks();

        last_tmr2_lsw = tmr2_data.tm_16[0];
 //     loop_cnt++;
        test_tmr2_lsw = tmr2_data.tm_16[0];
#if 1 
        // Make sure the timer read was atomic
        if (last_tmr2_lsw == test_tmr2_lsw)
        {
            switch (ledState)
            {
                case ON_1:      // ON_1 lasts 150 ms.
                    if (750 < (last_tmr2_lsw-led_cycle_start))
                    {
                        LED_Off (LED_GREEN);
                        ledState = OFF_1;
                    }
                    break;
                case OFF_1:      // OFF_1 lasts 150 ms.
                    if (1500 < (last_tmr2_lsw-led_cycle_start))
                    {
                        LED_On (LED_GREEN);
                        ledState = ON_2;
                    }
                    break;
                case ON_2:      // ON_1 lasts 150 ms.
                    if (2250 < (last_tmr2_lsw-led_cycle_start))
                    {
                        LED_Off (LED_GREEN);
                        ledState = OFF_2;
                    }
                    break;
                case OFF_2:      // OFF_2 lasts 550 ms.
                    if (5000 < (last_tmr2_lsw-led_cycle_start))
                    {
                        LED_On (LED_GREEN);
                        led_cycle_start = last_tmr2_lsw;
                        ledState = ON_1;
                    }
                    break;
                default:
                    LED_On (LED_GREEN);
                    led_cycle_start = last_tmr2_lsw;
                    ledState = ON_1;
            }
        }
#endif
#if 0
        loop_cnt++;
        if (led_off && (0 < loop_cnt) && (loop_cnt < 500))
        {
            LED_On (LED_GREEN);
            led_off = false;
        }
        else if (!led_off && (500 < loop_cnt) && (loop_cnt < 1000))
        {
            LED_Off (LED_GREEN);
            led_off = true;
        }
        else if (led_off && (1000 < loop_cnt) && (loop_cnt < 1500))
        {
            LED_On (LED_GREEN);
            led_off = false;
        }
        else if (!led_off && (1500 < loop_cnt) && (loop_cnt < 2500))
        {
            LED_Off (LED_GREEN);
            led_off = true;
        }
        else if (2500 < loop_cnt)
        {
            loop_cnt = 0;
        }
#endif

        
        #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif

        /* If the USB device isn't configured yet, we can't really do anything
         * else since we don't have a host to talk to.  So jump back to the
         * top of the while loop. */
        if( USBGetDeviceState() < CONFIGURED_STATE )
        {
            /* Jump back to the top of the while loop. */
            continue;
        }

        /* If we are currently suspended, then we need to see if we need to
         * issue a remote wakeup.  In either case, we shouldn't process any
         * keyboard commands since we aren't currently communicating to the host
         * thus just continue back to the start of the while loop. */
        if( USBIsDeviceSuspended() == true )
        {
            /* Jump back to the top of the while loop. */
            continue;
        }

        //Application specific tasks
        APP_DeviceCustomHIDTasks();

    }//end while
}//end main


bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch((int)event)
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            /* We are using the SOF as a timer to time the LED indicator.  Call
             * the LED update function here. */
            APP_LEDUpdateUSBStatus();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
            APP_LEDUpdateUSBStatus();
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
            APP_LEDUpdateUSBStatus();
            break;

        case EVENT_CONFIGURED:
            /* When the device is configured, we can (re)initialize the demo
             * code. */
            APP_DeviceCustomHIDInitialize();
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            /* We have received a non-standard USB request.  The HID driver
             * needs to check to see if the request was for it. */
            USBCheckHIDRequest();
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}

/*******************************************************************************
 End of File
*/

