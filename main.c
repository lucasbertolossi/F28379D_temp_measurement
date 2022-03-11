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

//****************************************************************************
//
// Include files
//
//****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "F28x_Project.h"
#include "F28379D_lcd.h"
#include "ADS124S08.h"
#include "adchal_tidrivers_adapted.h"
#include <math.h>
#include "Peripheral_Setup.h"
#include "SFO_V8.h"
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
    uint16_t EPwm_CMPA_Direction;
    uint16_t EPwm_CMPB_Direction;
    uint16_t EPwmTimerIntCount;
    uint16_t EPwmMaxCMPA;
    uint16_t EPwmMinCMPA;
    uint16_t EPwmMaxCMPB;
    uint16_t EPwmMinCMPB;
    float dutyCycle;
    float previousDutyCycle;
    bool flag_update_pwm;
}EPWM_INFO;
//    epwm2_info->EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
//    epwm2_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN; // and decreasing CMPB
EPWM_INFO epwm2_info = {
    .EPwmTimerIntCount = 0,               // Zero the interrupt counter
    .EPwmRegHandle = &EPwm2Regs,          // Set the pointer to the ePWM module
    .EPwmMaxCMPA = EPWM2_MAX_CMPA,        // Setup min/max CMPA/CMPB values
    .EPwmMinCMPA = EPWM2_MIN_CMPA,
    .EPwmMaxCMPB = EPWM2_MAX_CMPB,
    .EPwmMinCMPB = EPWM2_MIN_CMPB,
    .dutyCycle = 0.10,
    .previousDutyCycle = 0.00,
    .flag_update_pwm = false
};

//EPWM_INFO *epwm2_info = NULL;
//epwm2_info = &epwm2;

//float dutycycle;
uint16_t DutyFine;
uint16_t status;
uint16_t CMPA_reg_val;
uint16_t CMPAHR_reg_val;
uint16_t CMPB_reg_val;
uint16_t CMPBHR_reg_val;
uint32_t temp, temp1;
uint32_t MEP_ScaleFactor = 0; //scale factor value
//    volatile uint32_t ePWM[(PWM_CH + 1)] = {0, EPWM1_BASE, EPWM2_BASE};
volatile struct EPWM_REGS *ePWM[] = {0, &EPwm1Regs, &EPwm2Regs};


// Global parameter passing to interrupts must occur through global memory
char* sTemperature = NULL;
char* sRtdRes = NULL;
char* sSetpoint = NULL;
float rtdTemp = 0;
float rtdTempB = 0;
//float plot1[256];              // used for plotting temperature
<<<<<<< HEAD
//float plot2[1024];              // used for plotting temperature (testing)
//float plot3[1024];              // used for plotting temperature (testing)
=======
float plot2[1024];              // used for plotting temperature (testing)
//float plot3[1028];              // used for plotting temperature (testing)
>>>>>>> refs/remotes/origin/main
float *pTemperature = &rtdTemp; // used for plotting temperature
uint32_t index = 0;
uint32_t indextest = 0;
volatile uint16_t xINT1Count;
bool flag_nDRDY_INTERRUPT = false;
bool flag_update_SFO = false;

bool flag_pwm_right_on = false;
bool flag_pwm_left_on = false;
float setpoint = 25.0f;

typedef struct PIDController {
    /* Controller gains */
    const float Kp;
    const float Ki;
    const float Kd;
    /* Derivative low-pass filter time constant */
    const float tau;
    /* Output limits */
    const float limMin;
    const float limMax;
    /* Integrator limits */
//  float limMinInt;    Declared within function
//  float limMaxInt;
    /* Sample time (in seconds) */
    const float T;
    /* Controller "memory" */
    float integrator;
    float prevError;            /* Required for integrator */
    float differentiator;
    float prevMeasurement;      /* Required for differentiator */
    /* Controller output */
    float out;
    /* Setpoint */
//  float setpoint;
} PIDController;

PIDController pid = {
    .Kp      = 1.3f,
    .Ki      = 0.4f,
    .Kd      = 10.0f,
    .tau     = 0.02,
    .limMin  = -1.0,
    .limMax  = 1.0,
    .T       = 1,

    .integrator = 0.0,
    .prevError = 0.0,

    .differentiator = 0.0,
    .prevMeasurement = 0.0,

    .out = 0.0
};


/* Controller parameters */

//const float pidKp  = 1.0;
//const float pidKi  = 0.5;
//const float pidKd  = 0.0;
//const float pidTau = 0.02;
//const float pidLimMin = -1.0;
//const float pidLimMax = 1.0;
//const float sampleTimeS = 0.01;

//****************************************************************************
//
// Function Prototypes
//
//****************************************************************************
__interrupt void epwm2_isr(void);
__interrupt void xint1_isr(void);         // /DRDY
__interrupt void xint2_isr(void);         // Increase setpoint
__interrupt void xint3_isr(void);         // Decrease setpoint
__interrupt void cpu_timer0_isr(void);
void update_flag(EPWM_INFO *epwm_info);
void update_compare(EPWM_INFO *epwm_info);
void error(void);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
//****************************************************************************
//
// Main Function
//
//****************************************************************************
int main(void)
{

//
// Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();


// Initialize GPIO

   Setup_ePWM_Gpio();       // ePWM
   Setup_Buttons_Gpio();    // XINT2 XINT3
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
    PieVectTable.XINT2_INT = &xint2_isr;
    PieVectTable.XINT3_INT = &xint3_isr;
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
    ConfigCpuTimer(&CpuTimer0, 200, 100000);     // 100 ms timer
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
// Enable interrupts
//
    IER |= M_INT1;                            // Enable CPU INT1 (timer 0, XINT1, XINT2)
    IER |= M_INT13;                         // Enable CPU INT13 (timer 1)
//    IER |= M_INT14;                         // Enable CPU INT14 (timer 2)
    IER |= M_INT3;                            // Enable CPU INT3 (EPWM1-3 INT)
    IER |= M_INT12;                           // Enable CPU INT12 (XINT3)
//
// Enable XINT1 in the PIE: Group 1 interrupt 4
// Enable XINT2 in the PIE: Group 1 interrupt 5
// Enable XINT3 in the PIE: Group 12 interrupt 1
// Enable TINT0 in the PIE: Group 1 interrupt 7
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;      // XINT1
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;      // XINT2
    PieCtrlRegs.PIEIER12.bit.INTx1 = 1;     // XINT3
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
    DisplayLCD(1, "Initializing");
//    DisplayLCD(2, "Program");


//****************************************************************************
//
// Init PID controller
//
//****************************************************************************

    PIDController *controller = NULL;

    controller = &pid;

//****************************************************************************
//
// Initialize PWM
//
//****************************************************************************

    status = SFO_INCOMPLETE;
    DutyFine = 0;


////
//// Calling SFO() updates the HRMSTEP register with calibrated MEP_ScaleFactor.
//// HRMSTEP must be populated with a scale factor value prior to enabling
//// high resolution period control.
////
    while(status == SFO_INCOMPLETE)
    {
        status = SFO();
        if(status == SFO_ERROR)
        {
            error();   // SFO function returns 2 if an error occurs & # of MEP
        }              // steps/coarse step exceeds maximum of 255.
    }
//
    Setup_ePWM();



//    /* all below calculation apply for CMPB as well
//    // CMPA_reg_val , CMPA_reg_val is calculated as a Q0.
//    // Since DutyFine is a Q15 number, and the period is Q0
//    // the product is Q15. So to store as a Q0, we shift right
//    // 15 bits.
//
//    CMPA_reg_val = ((long)DutyFine * (EPwm1Regs.TBPRD + 1)) >> 15;
//
//    // This next step is to obtain the remainder which was
//    // truncated during our 15 bit shift above.
//    // compute the whole value, and then subtract CMPA_reg_val
//    // shifted LEFT 15 bits:
//    temp = ((long)DutyFine * (EPwm1Regs.TBPRD + 1)) ;
//    temp = temp - ((long)CMPA_reg_val<<15);
//
//    ** If auto-conversion is disabled, the following step can be
//    // skipped. If autoconversion is enabled, the SFO function will
//    // write the MEP_ScaleFactor to the HRMSTEP register and the
//    // hardware will automatically scale the remainder in the CMPAHR
//    // register by the MEP_ScaleFactor.
//    // Because the remainder calculated above (temp) is in Q15 format,
//    // it must be shifted left by 1 to convert to Q16 format for the
//    // hardware to properly convert.
//    CMPAHR_reg_val = temp<<1;
//
//    ** If auto-conversion is enabled, the following step is performed
//       automatically in hardware and can be skipped
//    // This obtains the MEP count in digits, from
//    // 0,1, .... MEP_Scalefactor.
//    // 0x0080 (0.5 in Q8) is converted to 0.5 in Q15 by shifting left 7.
//    // This is added to fractional duty*MEP_SF product in order to round
//    // the decimal portion of the product up to the next integer if the
//    // decimal portion is >=0.5.
//    //
//    //Once again since this is Q15
//    // convert to Q0 by shifting:
//    CMPAHR_reg_val = (temp*MEP_ScaleFactor+(0x0080<<7))>>15;
//
//    ** If auto-conversion is enabled, the following step is performed
//       automatically in hardware and can be skipped
//    // Now the lower 8 bits contain the MEP count.
//    // Since the MEP count needs to be in the upper 8 bits of
//    // the 16 bit CMPAHR register, shift left by 8.
//    CMPAHR_reg_val = CMPAHR_reg_val << 8;
//
//    // Write the values to the registers as one 32-bit or two 16-bits
//    EPwm1Regs.CMPA.bit.CMPA = CMPA_reg_val;
//    EPwm1Regs.CMPA.bit.CMPAHR = CMPAHR_reg_val;
//    */
//
//    //
//    // All the above operations may be condensed into
//    // the following form:
    DutyFine = ceil(32767*epwm2_info.dutyCycle);   // converts duty cycle (float) to Q15 number
    CMPA_reg_val = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) >> 15;
    CMPB_reg_val = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) >> 15;
    temp = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) ;
    temp1 = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) ;
    temp = temp - ((long)CMPA_reg_val << 15);
    temp1 = temp1 - ((long)CMPB_reg_val << 15);

    #if(AUTOCONVERT)
    CMPAHR_reg_val = temp << 1; // convert to Q16
    CMPBHR_reg_val = temp << 1; // convert to Q16
    #else
    CMPAHR_reg_val = ((temp * MEP_ScaleFactor) +
                      (0x0080 << 7)) >> 15;
    CMPAHR_reg_val = CMPAHR_reg_val << 8;
    CMPBHR_reg_val = ((temp1 * MEP_ScaleFactor) +
                      (0x0080 << 7)) >> 15;
    CMPBHR_reg_val = CMPBHR_reg_val << 8;
    #endif

   //
   // Example for a 32 bit write to CMPA:CMPAHR
   //
    EPwm2Regs.CMPA.all = ((long)CMPA_reg_val) << 16 |
                          CMPAHR_reg_val; // loses lower 8-bits

   //
   // Example for a 32 bit write to CMPB:CMPBHR
   //
    EPwm2Regs.CMPB.all = ((long)CMPB_reg_val) << 16 |
                          CMPBHR_reg_val; // loses lower 8-bits

//    //
//    // Call the scale factor optimizer lib function SFO()
//    // periodically to track for any change due to temp/voltage.
//    // This function generates MEP_ScaleFactor by running the
//    // MEP calibration module in the HRPWM logic. This scale
//    // factor can be used for all HRPWM channels. The SFO()
//    // function also updates the HRMSTEP register with the
//    // scale factor value.
//    //
    status = SFO(); // in background, MEP calibration module
                    // continuously updates MEP_ScaleFactor
//
    if(status == SFO_ERROR)
    {
        error();   // SFO function returns 2 if an error occurs & #
                   // of MEP steps/coarse step
    }              // exceeds maximum of 255.


    //****************************************************************************
    //
    // Initialize SPI and config ADS124S08
    //
    //****************************************************************************
    RTD_Set     *rtdSet = NULL;
    float       rtdRes, rtdTemp;
    ADCchar_Set adcChars;
    uint16_t    statusb;
    uint16_t    crc;
    char errorTimeOut[] = "Timeout on conv.";
    char errorSpiConfig[] = "Error in SPI";

    rtdSet = &PT100_RTD;

    adcChars.inputMuxConfReg = RTD_FOUR_WIRE_INPUT_MUX;
    adcChars.pgaReg          = RTD_FOUR_WIRE_PGA;
    adcChars.dataRateReg     = RTD_FOUR_WIRE_DATARATE;
    adcChars.refSelReg       = RTD_FOUR_WIRE_REF_SEL;
    adcChars.IDACmagReg      = RTD_FOUR_WIRE_IDACMAG;
    adcChars.IDACmuxReg      = RTD_FOUR_WIRE_IDACMUX;
    adcChars.Vref            = RTD_FOUR_WIRE_INT_VREF;
    rtdSet->Rref             = RTD_FOUR_WIRE_REF_RES;
    rtdSet->wiring           = Four_Wire_High_Side_Ref;

    adcChars.VBIASReg = RTD_VBIAS;

    // Initialize ADC peripherals, program ADC settings and check communication
    if ( !InitADCPeripherals(&adcChars) ) {
        DisplayLCD(1, errorSpiConfig);
        DisplayLCD(2, "");

        while (1);
    }


    //****************************************************************************
    //
    // Main Loop
    //
    //****************************************************************************

    while(1)
    {
        // Display setpoint in LCD
<<<<<<< HEAD
        floatToCharTemperature(setpoint, sSetpoint);
=======
//        floatToCharSetpoint(setpoint, sSetpoint);
//        floatToCharTemperature(setpoint, sSetpoint);
        floatToCharTemperature(25.000, sSetpoint);
>>>>>>> refs/remotes/origin/main
        DisplayLCD(2, sSetpoint);
        DisplayLCD(2, "Program");
    if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {

        adcChars.adcValue1 = readConvertedData( &statusb, &crc, COMMAND );

        // Convert ADC values RTD resistance
        rtdRes = Convert_Code2RTD_Resistance( &adcChars, rtdSet  );

        // Convert RTD resistance to temperature and linearize
//        rtdTemp = RTD_Linearization( rtdSet, rtdRes );
//        rtdTempB = RTD_Linearization( rtdSet, rtdRes );

        // Convert using calibration
        rtdTemp = calculate_temperature(rtdRes, 'A');
//        rtdTempB = calculate_temperature(rtdRes, 'E');

        /* Compute new control signal */
//        epwm2_info->dutyCycle = PIDController_Update(&pid, setpoint, rtdTemp);
        epwm2_info.dutyCycle = PIDController_Update(controller, setpoint, rtdTemp);

        if ( isnan(rtdTemp) ) {
            DisplayLCD(1, "NaN");
            DELAY_US(1000);
//            DisplayLCD(2, "");
        }
        else {
//            floatToChar(rtdTemp, sTemperature);
            floatToCharTemperature(rtdTemp, sTemperature);
            DisplayLCD(1, sTemperature);

//            floatToChar(rtdRes, sRtdRes);
//            DisplayLCD(2, sRtdRes);


<<<<<<< HEAD
//            plot2[indextest] = rtdTemp;     // graph A
=======
            plot2[indextest] = rtdTemp;     // graph A
>>>>>>> refs/remotes/origin/main
//            plot3[indextest] = rtdTempB;       // graph B
//            plot3[indextest] = rtdRes;

<<<<<<< HEAD
//            indextest = (indextest==1023) ? 0 : indextest+1;
=======
            indextest = (indextest==1023) ? 0 : indextest+1;
>>>>>>> refs/remotes/origin/main
        }
    } else {
        DisplayLCD(1, errorTimeOut);
        DisplayLCD(2, "");
//        while (1);
    }

    }   // end while loop
} //end main()



//****************************************************************************
//
// Functions / Interrupts
//
//****************************************************************************
//
// update_compare - Update the compare values for the specified EPWM
//
void update_compare(EPWM_INFO *epwm_info)
{
    float abs_duty_cycle;

//    if(flag_update_SFO)
//    {
//        flag_update_SFO = false;
//    }

    // Every 1s: change the CMPA/CMPB values
    if(epwm_info->flag_update_pwm)
    {
        epwm_info->flag_update_pwm = false;

        status = SFO();
        if(status == SFO_ERROR)
        {
        // SFO function returns 2 if an error occurs & # of
        // MEP steps/coarse step exceeds maximum of 255.
        error();
        }


        // makes duty cycle positive if control output is negative, checks if control needs to cool or heat system
        // Assuming pwm_left cools and pwm_right heats

        // Needs to cool the system (pwm_left)
        if(epwm_info->dutyCycle < 0){
            abs_duty_cycle = -epwm_info->dutyCycle;

            if(flag_pwm_right_on){
                enable_pwm_right(false);    // Clear GPIO18 low
                flag_pwm_right_on = false;
                epwm_info->previousDutyCycle = 0;
                DELAY_US(1000);     // delays for 1ms (dead time, could as low as rise/fall time of h bridge transistor)

                enable_pwm_left(true);
                flag_pwm_left_on = true;
            }

            if(!flag_pwm_left_on){
                enable_pwm_left(true);
                flag_pwm_left_on = true;
            }

        }else{
            abs_duty_cycle = epwm_info->dutyCycle;

            // Needs to heat the system (PWM_right)
            if(flag_pwm_left_on){
                enable_pwm_left(false);     // Clear GPIO19 low
                flag_pwm_left_on = false;
                epwm_info->previousDutyCycle = 0;

                DELAY_US(1000);     // delays for 1ms (dead time, could as low as rise/fall time of h bridge transistor)

                enable_pwm_right(true);
                flag_pwm_right_on = true;
            }

            if(!flag_pwm_right_on){
                enable_pwm_right(true);
                flag_pwm_right_on = true;
            }

        }

        // increases periodically the current and not so fast in the beginning (could be faster)
        if(abs_duty_cycle - epwm_info->previousDutyCycle > 0.1){
            abs_duty_cycle = epwm_info->previousDutyCycle + 0.1;
            epwm_info->previousDutyCycle = abs_duty_cycle;
        }

        // Calculates PWM duty cycle for HRPWM module and apply it
        DutyFine = ceil(32767*abs_duty_cycle);   // converts duty cycle (float) to Q15 number
        CMPA_reg_val = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) >> 15;
        CMPB_reg_val = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) >> 15;
        temp = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) ;
        temp1 = ((long)DutyFine * (EPwm2Regs.TBPRD + 1)) ;
        temp = temp - ((long)CMPA_reg_val << 15);
        temp1 = temp1 - ((long)CMPB_reg_val << 15);

        #if(AUTOCONVERT)
        CMPAHR_reg_val = temp << 1; // convert to Q16
        CMPBHR_reg_val = temp << 1; // convert to Q16
        #else
        CMPAHR_reg_val = ((temp * MEP_ScaleFactor) +
                         (0x0080 << 7)) >> 15;
        CMPAHR_reg_val = CMPAHR_reg_val << 8;
        CMPBHR_reg_val = ((temp1 * MEP_ScaleFactor) +
                         (0x0080 << 7)) >> 15;
        CMPBHR_reg_val = CMPBHR_reg_val << 8;
        #endif

        // Example for a 32 bit write to CMPA:CMPAHR
        EPwm2Regs.CMPA.all = ((long)CMPA_reg_val) << 16 |
                             CMPAHR_reg_val; // loses lower 8-bits

        // Example for a 32 bit write to CMPB:CMPBHR
        EPwm2Regs.CMPB.all = ((long)CMPB_reg_val) << 16 |
                             CMPBHR_reg_val; // loses lower 8-bits


    }
    return;
}


//
// error - Halt debugger when called
//
void error (void)
{
    ESTOP0;         // Stop here and handle error
}


float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    /*
    * Error signal
    */
    float error = setpoint - measurement;

    /*
    * Proportional
    */
    float proportional = pid->Kp * error;

    /*
    * Integral
    */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    float limMinInt; float limMaxInt;

    /* Compute integrator limits */
    if (pid->limMax > proportional){

        limMaxInt = pid->limMax - proportional;

    } else {

        limMaxInt = 0.0f;
    }

    if (pid->limMin < proportional){

        limMinInt = pid->limMin - proportional;

    } else {

        limMinInt = 0.0f;

    }

    /* Clamp integrator */
    if (pid->integrator > limMaxInt) {

        pid->integrator = limMaxInt;

    } else if (pid->integrator < limMinInt) {

        pid->integrator = limMinInt;
    }

    /*
    * Derivative (band-limited differentiator)
    * 1/(2tau+T) is missing on first term, but i only realized after i tweaked the parameters, so i will leave it like this
    */

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)   /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    /*
    * Compute output and apply limits
    */
    pid->out = proportional + pid->integrator + pid->differentiator;


    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

    //testing pid control
    if(pid->out>0){
        pid->out = (proportional + pid->integrator + pid->differentiator)/3;

        if (pid->out > pid->limMax) {

            pid->out = pid->limMax;

        } else if (pid->out < pid->limMin) {

            pid->out = pid->limMin;

        }

    }


    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}


// External interrupt XINT1 indicates availability of new conversion data.
__interrupt void xint1_isr(void)
{
    xINT1Count++; // Watch variable - checking if /DRDY is going low after each conversion

    /* Set nDRDY flag to true */
    flag_nDRDY_INTERRUPT = true;

    // Acknowledge this interrupt to get more interruptions from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// External interrupt XINT2 increases setpoint
__interrupt void xint2_isr(void)
{
    if (setpoint < 30){
        setpoint += 1;
    }

    // Acknowledge this interrupt to get more interruptions from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// External interrupt XINT3 decreases setpoint
__interrupt void xint3_isr(void)
{
    if (setpoint > 15){
        setpoint -= 1;
    }

    // Acknowledge this interrupt to get more interruptions from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}



// cpu_timer0_isr - CPU Timer0 ISR with interrupt counter - 100 ms timer
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    // Update SFO (HRPWM)
//    if(CpuTimer0.InterruptCount==5){
//        flag_update_SFO = true;
//    }

    if(CpuTimer0.InterruptCount==10){
        update_flag(&epwm2_info);
        CpuTimer0.InterruptCount= 0;
    }

   //
   // Acknowledge this interrupt to receive more interrupts from group 1
   //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


////
//// epwm2_isr - EPWM2 ISR to update compare values
////
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


void update_flag(EPWM_INFO *epwm_info){
    epwm_info->flag_update_pwm = true;
}


//
// End of file
//

