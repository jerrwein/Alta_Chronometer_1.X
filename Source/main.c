
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
#include <xc.h>

#include "system.h"
// USB #include <system_config.h>

//#include "usb.h"
//#include "usb_device_hid.h"

//#include "app_device_custom_hid.h"
//#include "app_led_usb_status.h"

MAIN_RETURN main(void)
{
    int delay_cnt;
    int junk;
   
#ifdef JMW_TIMER1_INT

    // Configure Timer1
    T1CONbits.TMR1ON = 0;   // Disable Timer1 initially
    T1CONbits.TMR1CS = 0;   // Select internal clock source (Fosc/4 = 4 MHz)
    T1CONbits.T1CKPS = 3;   // Prescaling (1:8) 1M/65535=7.630Hz
//    T1CONbits.T1CKPS = 2;   // Prescaling (1:4) 1M/65535=15.259Hz
//    T1CONbits.T1CKPS = 1;   // Prescaling (1:2) 2M/65535=30.518Hz
//    T1CONbits.T1CKPS = 0;   // No prescaling (1:1) 4M/65535=61.036Hz
    T1GCONbits.TMR1GE = 0;  // No Gate Enable

    // Clear TMR1 registers
    TMR1H = 0;
    TMR1L = 0;
 //    TMR1 = 0; // clear timer value
    // JMW Timer 1 End
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

#if 0
//    TRISBbits.TRISB4 = 1;
//    TRISBbits.TRISB5 = 1;
//    TRISBbits.TRISB6 = 1;
//    TRISBbits.TRISB7 = 1;

//    ANSELBbits.ANSB4 = 0;
//    ANSELBbits.ANSB5 = 0;

//    bool bVal4, bVal5, bVal6, bVal7;
//    while(1)
//    {
//        bVal4 = PORTBbits.RB4;
//        bVal5 = PORTBbits.RB5;
//        bVal6 = PORTBbits.RB6;
//        bVal7 = PORTBbits.RB7;
//    };
#endif

//    SYSTEM_Initialize(SYSTEM_STATE_USB_START);
    SYSTEM_Initialize();

// JMW No USB Yet    USBDeviceInit();
// JMW No USB Yet    USBDeviceAttach();

#ifdef JMW_TIMER1_INT
    // Enable Timer1
    T1CONbits.TMR1ON = 1;
    // Enable interrupts from Timer1
    PIE1bits.TMR1IE = 1;    // Enable Timer1 interrupt
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
#endif

#ifdef JMW_IOC_RB7_INT
    // Enable IOC interrupts
    INTCONbits.IOCIE = 1;
#endif

#if defined(JMW_TIMER1_INT) || defined(JMW_IOC_RB7_INT)
    // JMW - Enable global interrupts
    INTCONbits.GIE = 1;     // Enable global interrupts
#endif

    while(1)
    {
// JMW No USB Yet       SYSTEM_Tasks();
        
        LED1_LAT = LED_ON;  // 1st. pulse
        delay_cnt = 2000;      
        while (--delay_cnt)
        {
            junk = delay_cnt; 
        }
        LED1_LAT = LED_OFF;
        delay_cnt = 2000;
        while (--delay_cnt)
        {
            junk = delay_cnt; 
        }
        
        LED1_LAT = LED_ON;  // 2nd. pulse
        delay_cnt = 2000;      
        while (--delay_cnt)
        {
            junk = delay_cnt; 
        }
        LED1_LAT = LED_OFF;
       
        delay_cnt = 6000;
        while (--delay_cnt)
        {
            junk = delay_cnt; 
        }
        
        // Toggle Blue LED
        LED2_LAT ^= 1;
        
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
// JMW No USB Yet           if( USBGetDeviceState() < CONFIGURED_STATE )
// JMW No USB Yet           {
            /* Jump back to the top of the while loop. */
// JMW No USB Yet               continue;
// JMW No USB Yet           }

        /* If we are currently suspended, then we need to see if we need to
         * issue a remote wakeup.  In either case, we shouldn't process any
         * keyboard commands since we aren't currently communicating to the host
         * thus just continue back to the start of the while loop. */
// JMW No USB Yet           if( USBIsDeviceSuspended() == true )
//        {
//            /* Jump back to the top of the while loop. */
//            continue;
// JMW No USB Yet           }

        //Application specific tasks
// JMW No USB Yet           APP_DeviceCustomHIDTasks();

    }//end while
}//end main

// JMW No USB Yet
#if 0
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
#endif
/*******************************************************************************
 End of File
*/

