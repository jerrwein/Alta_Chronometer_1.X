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

#include <leds.h>
#include <stdbool.h>
#include <xc.h>

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

/*********************************************************************
* Function: void LED_On(LED led);
*
* Overview: Turns requested LED on
*
* PreCondition: LED configured via LED_Configure()
*
* Input: LED led - enumeration of the LEDs available in this
*        demo.  They should be meaningful names and not the names of
*        the LEDs on the silkscreen on the board (as the demo code may
*        be ported to other boards).
*         i.e. - LED_On(LED_CONNECTION_DETECTED);
*
* Output: none
*
********************************************************************/
void LED_On(LED led)
{
    switch(led)
    {
        case LED_GREEN:
            LED1_LAT = LED_ON;
            break;

        case LED_BLUE:
            LED2_LAT = LED_ON;
            break;
	
        case LED_NONE:
            break;
    }
}

/*********************************************************************
* Function: void LED_Off(LED led);
*
* Overview: Turns requested LED off
*
* PreCondition: LED configured via LEDConfigure()
*
* Input: LED led - enumeration of the LEDs available in this
*        demo.  They should be meaningful names and not the names of
*        the LEDs on the silkscreen on the board (as the demo code may
*        be ported to other boards).
*         i.e. - LED_Off(LED_CONNECTION_DETECTED);
*
* Output: none
*
********************************************************************/
void LED_Off(LED led)
{
    switch(led)
    {
        case LED_GREEN:
            LED1_LAT = LED_OFF;
            break;

        case LED_BLUE:
            LED2_LAT = LED_OFF;
            break;

        case LED_NONE:
            break;
    }
}

/*********************************************************************
* Function: void LED_Toggle(LED led);
*
* Overview: Toggles the state of the requested LED
*
* PreCondition: LED configured via LEDConfigure()
*
* Input: LED led - enumeration of the LEDs available in this
*        demo.  They should be meaningful names and not the names of
*        the LEDs on the silkscreen on the board (as the demo code may
*        be ported to other boards).
*         i.e. - LED_Toggle(LED_CONNECTION_DETECTED);
*
* Output: none
*
********************************************************************/
void LED_Toggle(LED led)
{
    switch(led)
    {
        case LED_GREEN:
            LED1_LAT ^= 1;
            break;

        case LED_BLUE:
            LED2_LAT ^= 1;
            break;

          case LED_NONE:
            break;
    }
}

/*********************************************************************
* Function: bool LED_Get(LED led);
*
* Overview: Returns the current state of the requested LED
*
* PreCondition: LED configured via LEDConfigure()
*
* Input: LED led - enumeration of the LEDs available in this
*        demo.  They should be meaningful names and not the names of
*        the LEDs on the silkscreen on the board (as the demo code may
*        be ported to other boards).
*         i.e. - LED_Get(LED_CONNECTION_DETECTED);
*
* Output: true if on, false if off
*
********************************************************************/
bool LED_Get(LED led)
{
    switch(led)
    {
        case LED_GREEN:
            return ( (LED1_LAT == LED_ON) ? true : false );

        case LED_BLUE:
            return ( (LED2_LAT == LED_ON) ? true : false );

        case LED_NONE:
            return false;
    }
    
    return false;
}

/*********************************************************************
* Function: void LED_Enable(LED led);
*
* Overview: Configures the LED for use by the other LED API
*
* PreCondition: none
*
* Input: LED led - enumeration of the LEDs available in this
*        demo.  They should be meaningful names and not the names of
*        the LEDs on the silkscreen on the board (as the demo code may
*        be ported to other boards).
*
* Output: none
*
********************************************************************/
void LED_Enable(LED led)
{
    switch(led)
    {
        case LED_GREEN:
            LED1_TRIS = PIN_OUTPUT;
            break;

        case LED_BLUE:
            LED2_TRIS = PIN_OUTPUT;
            break;
 
        case LED_NONE:
            break;
    }
}
