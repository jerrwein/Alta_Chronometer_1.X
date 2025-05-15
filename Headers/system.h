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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <xc.h>
#include <stdbool.h>

// #include "buttons.h"
// #include "leds.h"
// #include "adc.h"

//#include "io_mapping.h"
//#include "fixed_address_memory.h"
//#include "power.h"


// #define JMW_TIMER1_INT
// #define JMW_IOC_RB7_INT

// Green LED (LED1)
#define LED1_LAT LATCbits.LATC5
#define LED1_TRIS TRISCbits.TRISC5

// Blue LED (LED2)
#define LED2_LAT LATCbits.LATC2
#define LED2_TRIS TRISCbits.TRISC2

#define LED_ON  0
#define LED_OFF 1

#define PIN_INPUT  1
#define PIN_OUTPUT 0


//Internal oscillator option setting.  Uncomment if using HFINTOSC+active clock 
//tuning, instead of a crystal.  
#define USE_INTERNAL_OSC        //Make sure 1uF-8uF extra capacitance is added on VDD net
                                //to smooth VDD ripple from MAX3232 chip, before using this
                                //with the original Low Pin Count USB Development Kit board.
                                //If using the latest version of the board, this is not
                                //required and is already present.



#define MAIN_RETURN void

/*** System States **************************************************/
typedef enum
{
    SYSTEM_STATE_USB_START,
    SYSTEM_STATE_USB_SUSPEND,
    SYSTEM_STATE_USB_RESUME
} SYSTEM_STATE;

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
// void SYSTEM_Initialize( SYSTEM_STATE state );
void SYSTEM_Initialize( void );


/*********************************************************************
* Function: void SYSTEM_Tasks(void)
*
* Overview: Runs system level tasks that keep the system running
*
* PreCondition: System has been initalized with SYSTEM_Initialize()
*
* Input: None
*
* Output: None
*
********************************************************************/
//void SYSTEM_Tasks(void);
#define SYSTEM_Tasks()

#endif //SYSTEM_H
