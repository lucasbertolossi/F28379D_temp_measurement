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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "F28x_Project.h"
#include "F28379D_lcd.h"
#include "ADS124S08.h"
#include "adchal_tidrivers_adapted.h"
//#include "pin_map.h"
//#include "device.h"

//****************************************************************************
//
// Global
//
//****************************************************************************

//volatile uint32_t xINT1Count;
volatile uint16_t dataw;
// Debug messages to display in LCD
char message[] = "F28379D";

bool flag_nDRDY_INTERRUPT = false;

//
// Function Prototypes
//
__interrupt void xint1_isr(void);
__interrupt void cpu_timer0_isr(void);
//__interrupt void cpu_timer1_isr(void);
//__interrupt void cpu_timer2_isr(void);
//
// Main
//
int main(void)
{
//    Uint32 xINT1Count;

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
// Interrupts are re-mapped to ISR functions found within this project (DefaultISR.c)
//
    EALLOW;
    PieVectTable.XINT1_INT = &xint1_isr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
//    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
//    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    EDIS;

//
// Initialize the Device Peripheral. This function can be found
// in F2837xD_CpuTimers.c
//
    InitCpuTimers();   // For this example, only initialize the Cpu Timers


//
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 200MHz CPU Freq, 1 second Period (in uSeconds)
//
    ConfigCpuTimer(&CpuTimer0, 200, 1000000);
//    ConfigCpuTimer(&CpuTimer1, 200, 1000000);
//    ConfigCpuTimer(&CpuTimer2, 200, 1000000);


//
// To ensure precise timing, use write-only instructions to write to the
// entire register. Therefore, if any of the configuration bits are changed in
// ConfigCpuTimer and InitCpuTimers (in F2837xD_cputimervars.h), the below
// settings must also be updated.
//
    CpuTimer0Regs.TCR.all = 0x4000;
//    CpuTimer1Regs.TCR.all = 0x4000;
//    CpuTimer2Regs.TCR.all = 0x4000;


//
// Clear the counter
//
//    xINT1Count = 0;       // Count XINT1 interrupts


//
// Enable XINT1 in the PIE: Group 1 interrupt 4
// Enable INT1 which is connected to WAKEINT:
//
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    IER |= M_INT1;                              // Enable CPU INT1 (timer 0)
//    IER |= M_INT13;                         // Enable CPU INT13 (timer 1)
//    IER |= M_INT14;                         // Enable CPU INT14 (timer 2)


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
    DisplayLCD(1, "Initializing");
    DisplayLCD(2, "Program");

    // Initializes SPI and ADS124S08 communication
    readRTDtemp();

// - Test with global chop on
// - Test with CRC and status byte
//   Perform offset calibration before system gain calibration
//

    while(1)
    {

    }

}

// External interrupt XINT1 indicates availability of new conversion data.
__interrupt void xint1_isr(void)
{
//    xINT1Count++; // Watch variable - checking if /DRDY is going low after each conversion

    /* Set nDRDY flag to true */
    flag_nDRDY_INTERRUPT = true;

    //
    // Acknowledge this interrupt to get more interruptions from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

//
// cpu_timer0_isr - CPU Timer0 ISR with interrupt counter
//
__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;

   //
   // Acknowledge this interrupt to receive more interrupts from group 1
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr - CPU Timer1 ISR
//
//__interrupt void cpu_timer1_isr(void)
//{
//   CpuTimer1.InterruptCount++;
//}


//
//
// cpu_timer2_isr CPU Timer2 ISR
//
//__interrupt void cpu_timer2_isr(void)
//{
//   CpuTimer2.InterruptCount++;
//}



//
// End of file
//

