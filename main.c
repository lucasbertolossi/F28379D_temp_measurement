//###########################################################################
//
// FILE:   main.c
//
// TITLE:  F28379D with ADS124S08EVM integration
//
// This project should connect F28379D with ADS124S08EVM using SPI and display temperature measured on LCD.
// The connections with the display are presented in F28379D_lcd.h
// The connections with the ADS124S08EVM are presented in ADS124S08.h

// Copyright notice, list of conditions and disclaimer of support library
//###########################################################################
// $TI Release: F2837xD Support Library v3.12.00.00 $
// $Release Date: Fri Feb 12 19:03:23 IST 2021 $
// $Copyright:
// Copyright (C) 2013-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include <stdint.h>
#include "F28x_Project.h"
#include "Peripheral_Setup.h"
#include "F28379D_lcd.h"
#include "ADS124S08.h"
//#include "pin_map.h"
//#include "device.h"

//
// Globals
//
volatile uint32_t xINT1Count;
volatile uint16_t dataw;
// Debug messages to display in LCD
char message[] = "LAUNCHXL";
char message2[] = "F28379D";

//
// Function Prototypes
//
interrupt void xint1_isr(void);

//
// Main
//
int main(void)
{
    Uint32 xINT1Count;

//
// Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();

//
// Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file (DefaultISR.c).
//
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.XINT1_INT = &xint1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

//
// Clear the counter
//
    xINT1Count = 0;                             // Count XINT1 interrupts

//
// Enable XINT1 in the PIE: Group 1 interrupt 4
// Enable INT1 which is connected to WAKEINT:
//
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    IER |= M_INT1;                              // Enable CPU INT1


//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Initialize LCD
//
    Gpio_Setup_LCD();       // Enable GPIO for LCD as output pins on GPIO4 - GPIO11, GPIO14 - GPIO15;
    InitializeLCD();        // Initialize LCD;
    DisplayLCD(1, message);
    DisplayLCD(2, message2);

//
// Initializes ADS124S08 communication
// - Test with global chop on
// - Test with CRC and status byte
//    Perform offset calibration before system gain calibration
//
//    DELAY_US(2200);

//    InitSpiADS124S08();     // Configure SPI interface (CPOL = 0, CPHA =1);
//
//    InitSpiGpioADC();       // Initializes GPIO58 - GPIO61 as SPISIMOA, SPISOMIA, SPICLKA, SPISTEA.
//
//    SetupDRDYGpio();        // Sets GPIO0 as falling edge external interruption
//    three functions above replaced by initADCperipherals() in adchal file

    while(1)
    {


    }

}

// External interrupt XINT1 indicates availability of new conversion data.
interrupt void xint1_isr(void)
{
    xINT1Count++; // Watch variable - checking if /DRDY is going low after each conversion





    //
    // Acknowledge this interrupt to get more interruptions from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

//
// End of file
//
