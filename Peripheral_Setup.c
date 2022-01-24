/*
 * Peripheral_Setup.c
 *
 *  Created on: 23 de nov de 2021
 *      Author: lbert
 */

#include "Peripheral_Setup.h"


void Setup_ePWM(void){
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;             // PRD=50000 -> PWM freq = 1 kHz // Set timer period, 200e6/(2*freq) and /2 if updown
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;              // Phase = 0
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm2Regs.TBCTR = 0x0000;                        // Clear counter
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;       // Count UPDOWN
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;          // Disable Phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;         // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;            // Useful for PRD if you need low frequency
    // (for ex 20hz) and PRD can go up to 65500 only so you need to divide for /64 first


    // Shadow registers are used to update the registers.
    // Example, if CMPA is changed, it can be changed in CTR=ZERO/PRD/PRD&ZERO
    //
    // Setup shadow register load on ZERO
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
//    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
//    EPwm1Regs.CMPA.bit.CMPA = EPWM1_MIN_CMPA;     // Set compare A value
//    EPwm1Regs.CMPB.bit.CMPB = EPWM1_MIN_CMPB;     // Set Compare B value
    EPwm2Regs.CMPA.bit.CMPA = 5000; // PRD-CMPA (duty cycle = 10%). add input parameter to update duty cycle

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;            // Clear PWM2A on Period
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A,
                                                    // up count

    EPwm2Regs.AQCTLB.bit.PRD = AQ_CLEAR;            // Clear PWM2B on Period
    EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;              // Set PWM2B on event A,
                                                    // up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event


    // Enables PWM pair (EPwm2A/B) and guarantees delay time to prevent short circuit
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // Enables Dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;           // Active Hi Complementary (EPwm2B is inverted)
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;

    // Configure below depending on the transistor's current up and down time
    EPwm2Regs.DBFED.bit.DBFED = 100;
    EPwm2Regs.DBRED.bit.DBRED = 100;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}


//
//  Enables PWM CLK and PWM GPIO
//
void Setup_ePWM_Gpio(void){
    EALLOW;

    CpuSysRegs.PCLKCR2.bit.EPWM2=1;     // Enable EPWM2 CLK

    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;

    EDIS;
}


