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
#include <math.h>
#include "Peripheral_Setup.h"
//#include "pin_map.h"
//#include "device.h"

/* RTD related includes */
#include "inc/rtd.h"
#include "inc/rtd_tables.h"

//****************************************************************************
//
// Global
//
//****************************************************************************
typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
}EPWM_INFO;

EPWM_INFO epwm2_info;


// Global parameter passing to interrupts must occur through global memory
char* sTemperature = "";
char* sRtdRes = "";

float rtdTemp = 0;
float plot1[1024];              // used for plotting temperature
float *pTemperature = &rtdTemp; // used for plotting temperature

uint32_t index = 0;

volatile uint16_t xINT1Count;

bool flag_nDRDY_INTERRUPT = false;

typedef enum RTDExampleDef{
    RTD_2_Wire_Fig15,     // 2-Wire RTD example using ADS124S08 EVM, User's Guide Figure 15
    RTD_3_Wire_Fig14,     // 3-Wire RTD example using ADS124S08 EVM, User's Guide Figure 14
    RTD_4_Wire_Fig16,     // 4-Wire RTD example using ADS124S08 EVM, User's Guide Figure 16
} RTD_Example;
//
// Function Prototypes
//
__interrupt void epwm2_isr(void);
__interrupt void xint1_isr(void);
__interrupt void cpu_timer0_isr(void);
//__interrupt void cpu_timer1_isr(void);
//__interrupt void cpu_timer2_isr(void);
void update_compare(EPWM_INFO *epwm_info);


//
// Main
//
int main(void)
{

//
// Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();


//
// Initialize PWM GPIO
//
   Setup_ePWM_Gpio();


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
    PieVectTable.EPWM2_INT = &epwm2_isr;
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
    xINT1Count = 0;       // Count XINT1 interrupts


//
// Enable INT1 which is connected to WAKEINT:
//


    IER |= M_INT1;                            // Enable CPU INT1 (timer 0)
//    IER |= M_INT13;                         // Enable CPU INT13 (timer 1)
//    IER |= M_INT14;                         // Enable CPU INT14 (timer 2)
    IER |= M_INT3;                            // Enable CPU INT3 (EPWM1-3 INT)
//
// Enable XINT1 in the PIE: Group 1 interrupt 4
// Enable TINT0 in the PIE: Group 1 interrupt 7
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;      // XINT1
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // TINT0
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;      // EPWM2

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Initialize CRC table for faster calculations
//
//    crcInit();

//
// Initialize LCD
//
    Gpio_Setup_LCD();       // Enable GPIO for LCD as output pins on GPIO4 - GPIO11, GPIO14 - GPIO15;
    InitializeLCD();        // Initialize LCD;
//    DisplayLCD(1, "Initializing");
//    DisplayLCD(2, "Program");

//
// Initialize PWM
//
    Setup_ePWM();

    //
    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
    epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt
                                                    // counter
    epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the
                                                    // ePWM module
    epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;        // Setup min/max
                                                    // CMPA/CMPB values
    epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
//    epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
//    epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;

// - Test with CRC and status byte
//   Perform offset calibration before system gain calibration
//
    // Initializes SPI and ADS124S08 communication

    RTD_Set     *rtdSet = NULL;
    RTD_Type    rtdType = Pt;
    RTD_Example rtdExample = RTD_4_Wire_Fig16;
    float       rtdRes, rtdTemp;
    ADCchar_Set adcChars;
    uint16_t    status;
    uint16_t    crc;
    char errorTimeOut[] = "Timeout on conv.";
    char errorSpiConfig[] = "Error in SPI";

//    switch ( rtdType ) {
//        case Pt:
    rtdSet = &PT100_RTD;
//            break;
//    }

//    switch ( rtdExample ) {
//        case RTD_4_Wire_Fig16:
    adcChars.inputMuxConfReg = RTD_FOUR_WIRE_INPUT_MUX;
    adcChars.pgaReg          = RTD_FOUR_WIRE_PGA;
    adcChars.dataRateReg     = RTD_FOUR_WIRE_DATARATE;
    adcChars.refSelReg       = RTD_FOUR_WIRE_REF_SEL;
    adcChars.IDACmagReg      = RTD_FOUR_WIRE_IDACMAG;
    adcChars.IDACmuxReg      = RTD_FOUR_WIRE_IDACMUX;
    adcChars.Vref            = RTD_FOUR_WIRE_INT_VREF;
    rtdSet->Rref             = RTD_FOUR_WIRE_REF_RES;
    rtdSet->wiring           = Four_Wire_High_Side_Ref;
//            break;
//    }
    adcChars.VBIASReg = RTD_VBIAS;

    // Initialize ADC peripherals, program ADC settings and check communication
    if ( !InitADCPeripherals(&adcChars) ) {
        DisplayLCD(1, errorSpiConfig);
        DisplayLCD(2, "");

        while (1);
    }


//
// MAIN LOOP
//
    while(1)
    {


    if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {

        adcChars.adcValue1 = readConvertedData( &status, &crc, COMMAND );

        // Convert ADC values RTD resistance
        rtdRes = Convert_Code2RTD_Resistance( &adcChars, rtdSet  );

        // Convert RTD resistance to temperature and linearize
        rtdTemp = RTD_Linearization( rtdSet, rtdRes );

        if ( isnan(rtdTemp) ) {
            DisplayLCD(1, "NaN");
            DELAY_US(1000);
            DisplayLCD(2, "");
        }
        else {
            floatToChar(rtdTemp, sTemperature);
            DisplayLCD(1, sTemperature);

            floatToChar(rtdRes, sRtdRes);
            DisplayLCD(2, sRtdRes);

            plot1[index] = *pTemperature;
            index = (index==1023) ? 0 : index+1;
        }
    } else {
        DisplayLCD(1, errorTimeOut);
        DisplayLCD(2, "");
        while (1);
    }



    }

}

//
// update_compare - Update the compare values for the specified EPWM
//
void update_compare(EPWM_INFO *epwm_info)
{
   //
   // Every 5'th interrupt, change the CMPA/CMPB values
   //
   if(epwm_info->EPwmTimerIntCount == 5)
   {
       epwm_info->EPwmTimerIntCount = 0;

       //
       // If we were increasing CMPA, check to see if
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
       //
//       if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
//       {
//           if(epwm_info->EPwmRegHandle->CMPA.bit.CMPA < epwm_info->EPwmMaxCMPA)
//           {
//              epwm_info->EPwmRegHandle->CMPA.bit.CMPA++;
//           }
//           else
//           {
//              epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
//              epwm_info->EPwmRegHandle->CMPA.bit.CMPA--;
//           }
//       }

       //
       // If we were decreasing CMPA, check to see if
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
       //
//       else
//       {
//           if(epwm_info->EPwmRegHandle->CMPA.bit.CMPA == epwm_info->EPwmMinCMPA)
//           {
//              epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
//              epwm_info->EPwmRegHandle->CMPA.bit.CMPA++;
//           }
//           else
//           {
//              epwm_info->EPwmRegHandle->CMPA.bit.CMPA--;
//           }
//       }

   }
   else
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}


// External interrupt XINT1 indicates availability of new conversion data.
__interrupt void xint1_isr(void)
{
    xINT1Count++; // Watch variable - checking if /DRDY is going low after each conversion

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

//    floatToChar(rtdTemp,sTemperature);
//    DisplayLCD(1, sTemperature);

//    plot1[index] = *pTemperature;
//    index = (index==1023) ? 0 : index+1;
   //
   // Acknowledge this interrupt to receive more interrupts from group 1
   //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//
// epwm2_isr - EPWM2 ISR to update compare values
//
__interrupt void epwm2_isr(void)
{
    //
    // Update the CMPA values
    //
    update_compare(&epwm2_info);

    //
    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


//
// cpu_timer1_isr - CPU Timer1 ISR
//
//__interrupt void cpu_timer1_isr(void)
//{
//    CpuTimer1.InterruptCount++;
//}


//
//
// cpu_timer2_isr CPU Timer2 ISR
//
//__interrupt void cpu_timer2_isr(void)
//{
//    CpuTimer2.InterruptCount++;
//}
//


//
// End of file
//

