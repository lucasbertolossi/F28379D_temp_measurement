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

    EPwm2Regs.TBPRD = 2000;                         // Set timer period
    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD >> 1; // PRD/2 (duty cycle = 50%) add input parameter
    EPwm2Regs.TBPHS.bit.TBPHS = 0;                  // Phase = 0
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count UPDOWN
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable Phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;           // Works with PRD if you need low frequency
    // (for ex 20hz) and PRD can go up to 65500 only so you need to divide for /64 first

    // Shadow registers are used to update the registers.
    // Example, if CMPA is changed, it can be changed in CTR=ZERO/PRD/PRD&ZERO
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   // Updates every half PWM cycle
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    //
    EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;                // CTR>CMPA = clear
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;                  // CTR>CMPA = set

    // Enables PWM pair (EPwm2A/B) and guarantees delay time to prevent short circuit
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;           // Active Hi Complementary (EPwm2B is inverted)
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // Enables Dead-band module

    // Configure below depending on the transistor's current up and down time
    EPwm2Regs.DBFED.bit.DBFED = 100;                     // FED = 20 TBCLKs

    EPwm2Regs.DBRED.bit.DBRED = 100;                     // RED = 20 TBCLKs

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Setup_ePWM_Gpio(void){
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
    EDIS;
}

