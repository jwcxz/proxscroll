//*****************************************************************************
//
// usb_dev_mouse.c - Main routines for the enumeration example.
//
// Copyright (c) 2009-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is a modification of part of revision 9453 of the EK-LM3S9D90 Firmware Package.
// Tweaked for the Stellaris Launchpad by Jeff Lawrence
//
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmouse.h"
#include "utils/uartstdio.h"
#include "usb_mouse_structs.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB HID Mouse Device (usb_dev_mouse)</h1>
//!
//! This example application turns the evaluation board into a USB mouse
//! supporting the Human Interface Device class.  The mouse pointer will move
//! in a square pattern for the duration of the time it is plugged in.
//!
//! UART0, connected to the FTDI virtual COM port and running at 115,200,
//! 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The incremental update for the mouse.
//
//*****************************************************************************
#define MOUSE_MOVE_INC          5
#define MOUSE_MOVE_DEC          -5

//*****************************************************************************
//
// The system tick timer rate.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100
#define MS_PER_SYSTICK          (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// Holds command bits used to signal the main loop to perform various tasks.
//
//*****************************************************************************
volatile unsigned long g_ulCommands;
#define TICK_EVENT              0

//*****************************************************************************
//
// A flag used to indicate whether or not we are currently connected to the USB
// host.
//
//*****************************************************************************
volatile tBoolean g_bConnected;

//*****************************************************************************
//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
//*****************************************************************************
volatile unsigned long g_ulSysTickCount;

//*****************************************************************************
//
// The number of system ticks to wait for each USB packet to be sent before
// we assume the host has disconnected.  The value 50 equates to half a second.
//
//*****************************************************************************
#define MAX_SEND_DELAY          50


unsigned long adc_buffer[4];


void adc_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 0);
    ADCIntClear(ADC0_BASE, 0);
}


void adc_get(unsigned long adc_buffer[]) {
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false));
    ADCSequenceDataGet(ADC0_BASE, 0, adc_buffer);
}


enum state {
    none,
    yellow,
    green,
    blue
};

enum state last = none;

#define idle 0
#define cw   1
#define ccw  -1
#define SAMPLE_THRESH 0xA0

char process_sample(unsigned long samples[]) {
    unsigned char i = 0;
    enum state curr = none;
    char axn = 0;

    // determine current position based on max value of each (and ensure
    // that the value is above the threshold)
    if ( samples[0] > SAMPLE_THRESH && 
            samples[0] > samples[1] && 
            samples[0] > samples[2] ) {
        curr = yellow;
    } else if ( samples[1] > SAMPLE_THRESH && 
            samples[1] > samples[0] && 
            samples[1] > samples[2] ) {
        curr = blue;
    } else if ( samples[2] > SAMPLE_THRESH && 
            samples[2] > samples[0] && 
            samples[2] > samples[1] ) {
        curr = green;
    } else {
        curr = none;
    }

    switch (last) {
        case none:
            // no previous information known
            break;

        case yellow:
            switch (curr) {
                case none:
                    axn = idle;
                    break;
                case yellow:
                    axn = idle;
                    break;
                case blue:
                    axn = cw;
                    break;
                case green:
                    axn = ccw;
                    break;
            }
            break;

        case blue:
            switch (curr) {
                case none:
                    axn = idle;
                    break;
                case yellow:
                    axn = ccw;
                    break;
                case blue:
                    axn = idle;
                    break;
                case green:
                    axn = cw;
                    break;
            }
            break;

        case green:
            switch (curr) {
                case none:
                    axn = idle;
                    break;
                case yellow:
                    axn = cw;
                    break;
                case blue:
                    axn = ccw;
                    break;
                case green:
                    axn = idle;
                    break;
            }
            break;
    }

    if ( last == none || curr != none ) {
        last = curr;
    }

    return axn;

}



//*****************************************************************************
//
// This enumeration holds the various states that the mouse can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    MOUSE_STATE_UNCONFIGURED,

    //
    // Nothing to send and not waiting on data.
    //
    MOUSE_STATE_IDLE,

    //
    // Waiting on data to be sent out.
    //
    MOUSE_STATE_SENDING
}
g_eMouseState = MOUSE_STATE_UNCONFIGURED;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// This function handles notification messages from the mouse device driver.
//
//*****************************************************************************
unsigned long
MouseHandler(void *pvCBData, unsigned long ulEvent,
             unsigned long ulMsgData, void *pvMsgData)
{
    switch(ulEvent)
    {
        //
        // The USB host has connected to and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            g_bConnected = true;

            break;
        }

        //
        // The USB host has disconnected from the device.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bConnected = false;
            g_eMouseState = MOUSE_STATE_UNCONFIGURED;

            break;
        }

        //
        // A report was sent to the host.  We are not free to send another.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            break;
        }

    }

    return(0);
}

//***************************************************************************
//
// Wait for a period of time for the state to become idle.
//
// \param ulTimeoutTick is the number of system ticks to wait before
// declaring a timeout and returning \b false.
//
// This function polls the current keyboard state for ulTimeoutTicks system
// ticks waiting for it to become idle.  If the state becomes idle, the
// function returns true.  If it ulTimeoutTicks occur prior to the state
// becoming idle, false is returned to indicate a timeout.
//
// \return Returns \b true on success or \b false on timeout.
//
//***************************************************************************
tBoolean
WaitForSendIdle(unsigned long ulTimeoutTicks)
{
    unsigned long ulStart;
    unsigned long ulNow;
    unsigned long ulElapsed;

    ulStart = g_ulSysTickCount;
    ulElapsed = 0;

    while(ulElapsed < ulTimeoutTicks)
    {
        //
        // Is the mouse is idle, return immediately.
        //
        if(g_eMouseState == MOUSE_STATE_IDLE)
        {
            return(true);
        }

        //
        // Determine how much time has elapsed since we started waiting.  This
        // should be safe across a wrap of g_ulSysTickCount.
        //
        ulNow = g_ulSysTickCount;
        ulElapsed = ((ulStart < ulNow) ? (ulNow - ulStart) :
                     (((unsigned long)0xFFFFFFFF - ulStart) + ulNow + 1));
    }

    //
    // If we get here, we timed out so return a bad return code to let the
    // caller know.
    //
    return(false);
}

//*****************************************************************************
//
// This function provides simulated movements of the mouse.
//
//*****************************************************************************
void
MoveHandler(char action)
{
    unsigned long ulRetcode;
    char cDeltaX, cDeltaY;

    //
    // Determine the direction to move the mouse.
    //
    ulRetcode = g_ulSysTickCount % (4 * SYSTICKS_PER_SECOND);
    if(ulRetcode < SYSTICKS_PER_SECOND)
    {
        cDeltaX = MOUSE_MOVE_INC;
        cDeltaY = 0;
    }
    else if(ulRetcode < (2 * SYSTICKS_PER_SECOND))
    {
        cDeltaX = 0;
        cDeltaY = MOUSE_MOVE_INC;
    }
    else if(ulRetcode < (3 * SYSTICKS_PER_SECOND))
    {
        cDeltaX = (char)MOUSE_MOVE_DEC;
        cDeltaY = 0;
    }
    else
    {
        cDeltaX = 0;
        cDeltaY = (char)MOUSE_MOVE_DEC;
    }

    //
    // Tell the HID driver to send this new report.
    //
    g_eMouseState = MOUSE_STATE_SENDING;
    //ulRetcode = USBDHIDMouseStateChange((void *)&g_sMouseDevice, cDeltaX, cDeltaY, 0);
    ulRetcode = USBDHIDMouseStateChange((void *)&g_sMouseDevice, 0, 0, 0, 0, action);

    //
    // Did we schedule the report for transmission?
    //
    if(ulRetcode == MOUSE_SUCCESS)
    {
        //
        // Wait for the host to acknowledge the transmission if all went well.
        //
        if(!WaitForSendIdle(MAX_SEND_DELAY))
        {
            //
            // The transmission failed, so assume the host disconnected and go
            // back to waiting for a new connection.
            //
            g_bConnected = false;
        }
    }
}

//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is called
// periodically and updates a global tick counter then sets a flag to tell the
// main loop to move the mouse.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    g_ulSysTickCount++;
    HWREGBITW(&g_ulCommands, TICK_EVENT) = 1;
}

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    char action, last_action = 0;

	ROM_FPULazyStackingEnable();
    //
    // Set the clocking to run from the PLL at 50MHz.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //
    // Enable the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioInit(0);
    UARTprintf("\033[2JMouse device application started\n");


	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2);

    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    //
    // Configure USB pins
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Set the system tick to fire 100 times per second.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, USB_MODE_DEVICE, 0);

    //
    // Pass the USB library our device information, initialize the USB
    // controller and connect the device to the bus.
    //
    USBDHIDMouseInit(0, (tUSBDHIDMouseDevice *)&g_sMouseDevice);

    adc_init();

    //
    // Drop into the main loop.
    //
    while(1)
    {
        //
        // Tell the user what we are doing.
        //
        UARTprintf("Waiting for host...\n");

        //
        // Wait for USB configuration to complete.
        //
        while(!g_bConnected)
        {
        }

        //
        // Update the status.
        //
        UARTprintf("Host connected...\n");

        //
        // Now keep processing the mouse as long as the host is connected.
        //
        while(g_bConnected)
        {
            //
            // If it is time to move the mouse then do so.
            //
            if(HWREGBITW(&g_ulCommands, TICK_EVENT) == 1)
            {
                HWREGBITW(&g_ulCommands, TICK_EVENT) = 0;
                adc_get(adc_buffer);
                action = process_sample(&(adc_buffer[1]));

                if ( action || last_action ) {
                    MoveHandler(action);
                    last_action = action;
                }
            }

        }

        //
        // Update the status.
        //
        UARTprintf("Host disconnected...\n");
    }
}
