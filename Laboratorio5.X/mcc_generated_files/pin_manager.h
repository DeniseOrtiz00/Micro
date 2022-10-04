/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F57Q43
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set LED1 aliases
#define LED1_TRIS                 TRISAbits.TRISA0
#define LED1_LAT                  LATAbits.LATA0
#define LED1_PORT                 PORTAbits.RA0
#define LED1_WPU                  WPUAbits.WPUA0
#define LED1_OD                   ODCONAbits.ODCA0
#define LED1_ANS                  ANSELAbits.ANSELA0
#define LED1_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define LED1_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define LED1_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define LED1_GetValue()           PORTAbits.RA0
#define LED1_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define LED1_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define LED1_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define LED1_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define LED1_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define LED1_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define LED1_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define LED1_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set LED2 aliases
#define LED2_TRIS                 TRISAbits.TRISA1
#define LED2_LAT                  LATAbits.LATA1
#define LED2_PORT                 PORTAbits.RA1
#define LED2_WPU                  WPUAbits.WPUA1
#define LED2_OD                   ODCONAbits.ODCA1
#define LED2_ANS                  ANSELAbits.ANSELA1
#define LED2_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define LED2_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define LED2_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define LED2_GetValue()           PORTAbits.RA1
#define LED2_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define LED2_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define LED2_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define LED2_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define LED2_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define LED2_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define LED2_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define LED2_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set LED3 aliases
#define LED3_TRIS                 TRISAbits.TRISA2
#define LED3_LAT                  LATAbits.LATA2
#define LED3_PORT                 PORTAbits.RA2
#define LED3_WPU                  WPUAbits.WPUA2
#define LED3_OD                   ODCONAbits.ODCA2
#define LED3_ANS                  ANSELAbits.ANSELA2
#define LED3_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define LED3_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define LED3_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define LED3_GetValue()           PORTAbits.RA2
#define LED3_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define LED3_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define LED3_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define LED3_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define LED3_SetPushPull()        do { ODCONAbits.ODCA2 = 0; } while(0)
#define LED3_SetOpenDrain()       do { ODCONAbits.ODCA2 = 1; } while(0)
#define LED3_SetAnalogMode()      do { ANSELAbits.ANSELA2 = 1; } while(0)
#define LED3_SetDigitalMode()     do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set LED4 aliases
#define LED4_TRIS                 TRISAbits.TRISA3
#define LED4_LAT                  LATAbits.LATA3
#define LED4_PORT                 PORTAbits.RA3
#define LED4_WPU                  WPUAbits.WPUA3
#define LED4_OD                   ODCONAbits.ODCA3
#define LED4_ANS                  ANSELAbits.ANSELA3
#define LED4_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define LED4_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define LED4_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define LED4_GetValue()           PORTAbits.RA3
#define LED4_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define LED4_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define LED4_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define LED4_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define LED4_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define LED4_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define LED4_SetAnalogMode()      do { ANSELAbits.ANSELA3 = 1; } while(0)
#define LED4_SetDigitalMode()     do { ANSELAbits.ANSELA3 = 0; } while(0)

// get/set LED5 aliases
#define LED5_TRIS                 TRISAbits.TRISA4
#define LED5_LAT                  LATAbits.LATA4
#define LED5_PORT                 PORTAbits.RA4
#define LED5_WPU                  WPUAbits.WPUA4
#define LED5_OD                   ODCONAbits.ODCA4
#define LED5_ANS                  ANSELAbits.ANSELA4
#define LED5_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define LED5_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define LED5_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define LED5_GetValue()           PORTAbits.RA4
#define LED5_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define LED5_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define LED5_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define LED5_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define LED5_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define LED5_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define LED5_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define LED5_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set LED6 aliases
#define LED6_TRIS                 TRISAbits.TRISA5
#define LED6_LAT                  LATAbits.LATA5
#define LED6_PORT                 PORTAbits.RA5
#define LED6_WPU                  WPUAbits.WPUA5
#define LED6_OD                   ODCONAbits.ODCA5
#define LED6_ANS                  ANSELAbits.ANSELA5
#define LED6_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LED6_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LED6_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LED6_GetValue()           PORTAbits.RA5
#define LED6_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LED6_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LED6_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define LED6_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define LED6_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define LED6_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define LED6_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define LED6_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set LED7 aliases
#define LED7_TRIS                 TRISAbits.TRISA6
#define LED7_LAT                  LATAbits.LATA6
#define LED7_PORT                 PORTAbits.RA6
#define LED7_WPU                  WPUAbits.WPUA6
#define LED7_OD                   ODCONAbits.ODCA6
#define LED7_ANS                  ANSELAbits.ANSELA6
#define LED7_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define LED7_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define LED7_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define LED7_GetValue()           PORTAbits.RA6
#define LED7_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define LED7_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define LED7_SetPullup()          do { WPUAbits.WPUA6 = 1; } while(0)
#define LED7_ResetPullup()        do { WPUAbits.WPUA6 = 0; } while(0)
#define LED7_SetPushPull()        do { ODCONAbits.ODCA6 = 0; } while(0)
#define LED7_SetOpenDrain()       do { ODCONAbits.ODCA6 = 1; } while(0)
#define LED7_SetAnalogMode()      do { ANSELAbits.ANSELA6 = 1; } while(0)
#define LED7_SetDigitalMode()     do { ANSELAbits.ANSELA6 = 0; } while(0)

// get/set LED8 aliases
#define LED8_TRIS                 TRISAbits.TRISA7
#define LED8_LAT                  LATAbits.LATA7
#define LED8_PORT                 PORTAbits.RA7
#define LED8_WPU                  WPUAbits.WPUA7
#define LED8_OD                   ODCONAbits.ODCA7
#define LED8_ANS                  ANSELAbits.ANSELA7
#define LED8_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define LED8_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define LED8_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define LED8_GetValue()           PORTAbits.RA7
#define LED8_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define LED8_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define LED8_SetPullup()          do { WPUAbits.WPUA7 = 1; } while(0)
#define LED8_ResetPullup()        do { WPUAbits.WPUA7 = 0; } while(0)
#define LED8_SetPushPull()        do { ODCONAbits.ODCA7 = 0; } while(0)
#define LED8_SetOpenDrain()       do { ODCONAbits.ODCA7 = 1; } while(0)
#define LED8_SetAnalogMode()      do { ANSELAbits.ANSELA7 = 1; } while(0)
#define LED8_SetDigitalMode()     do { ANSELAbits.ANSELA7 = 0; } while(0)

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



#endif // PIN_MANAGER_H
/**
 End of File
*/