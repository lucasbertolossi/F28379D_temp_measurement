/*
 * adchal_tidrivers_adapted.c
 *
 * * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Software License Agreement
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's Microcontroller and/or Data Converter products.
 * The software is owned by TI and/or its suppliers, and is protected
 * under applicable copyright laws.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * @brief Hardware Abstraction Layer for ADS124S08EVM <-> C2000 launchpad -F28379D
 * Adapted from @file adchal_TIDrivers.c
 *
 *  Created on: 1 de nov de 2021
 *  Author: Lucas Bertolossi
 */


#include <stdint.h>
#include "F28x_Project.h"


//****************************************************************************
//
// Internal variables and macros
//
//****************************************************************************

// Flag to indicate if an interrupt occured
extern volatile bool flag_nDRDY_INTERRUPT;

/************************************************************************************//**
 *
 * @brief gpioDRDYfxn()
 *          MCU callback routine for ADC_DRDY falling edge interrupt
 *
 * @return None
 */
void ADC_DRDY_CallbackFxn( uint_least8_t index )
{
    /* Set nDRDY flag to true */
    flag_nDRDY_INTERRUPT = true;
}



/************************************************************************************//**
 *
 * @brief InitSpiADS124S08()
 *          This function initializes the SPI to a known state to work with ADS124S08EVM
 *
 * @param[in]   -
 *
 * @return   None
 *
 * @code
 *
 * @endcode
 */
void InitSpiADS124S08(void)
{
    // Step 1. Clear the SPI Software Reset bit (SPISWRESET) to 0 to force the SPI to the reset state.
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    // Step 2. Configure the SPI as desired:
    // • Select either master or slave mode (MASTER_SLAVE).
    //   Enable master (0 == slave, 1 == master)
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;

    // • Choose SPICLK polarity and phase (CLKPOLARITY and CLK_PHASE). SPI mode 1 selected.
    //   Clock polarity (0 == rising, 1 == falling), ADS124S08 operates in SPI mode 1 (CPOL = 0, CPHA = 1)
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    //   Clock phase (0 == normal, 1 == delayed)
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

    // • Set the desired baud rate (SPIBRR).
    //   ADS124S08 - Minimum period for SCLK = 100 ns (maximum frequency is 10 MHz)
    //   Baud rate for HS_MODE = 0 is LSPCLK/(SPIBRR+1) (example 18-3)
    //   Default SYSCLK = 200 MHz
    //   LSPCLKDIV is set to default /4 table 3-164 page 345 (LOSPCP register field)
    //   LSPCLK = 200 / 4 = 50 MHz, SPIBRR = 49 was chosen -> baud rate = 1 MHz
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR_ADS124S08;

    // • Enable high speed mode if desired (HS_MODE).  (Not needed)
    // • Set the SPI character length (SPICHAR) to 8.
    SpiaRegs.SPICCR.bit.SPICHAR = (SPI_WORD_SIZE-1);

    // • Clear the SPI Flags (OVERRUN_FLAG, INT_FLAG). Cleared setting SPISWRESET to 0
//    SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;

    // • Enable SPISTE inversion (STEINV) if needed. (Not needed)
    // • Enable 3-wire mode (TRIWIRE) if needed. (Not needed)

    // • If using FIFO enhancements:
    // – Enable the FIFO enhancements (SPIFFENA). > SPIFFTX.14 = 1
    // – Clear the FIFO Flags (TXFFINTCLR, RXFFOVFCLR, and RXFFINTCLR).
    //   SPIFFTX.6 = 1 / SPIFFRX.14 = 1 / SPIFFRX.6 = 1
    // – Release transmit and receive FIFO resets (TXFIFO and RXFIFORESET).
    //   SPIFFTX.13 = 1 / SPIFFRX.13 = 1
    // – Release SPI FIFO channels from reset (SPIRST).
    //   SPIFFTX.15 = 1
    // Step 3. If interrupts are used:
    // • In non-FIFO mode, enable the receiver overrun and/or SPI interrupts (OVERRUNINTENA
    // and SPIINTENA).
    // • In FIFO mode, set the transmit and receive interrupt levels (TXFFIL and RXFFIL) then
    // enable the interrupts (TXFFIENA and RXFFIENA). (bits 4-0)
    // SPI interrupts are disabled (SPICTL.SPIINTENA) by default // FIFO interrupts are disabled
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2040;

    // Enable transmission (Talk)
    SpiaRegs.SPICTL.bit.TALK = 1;

    // Step 4. Set SPISWRESET to 1 to release the SPI from the reset state.
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI (debugging purposes)
    SpiaRegs.SPIPRI.bit.FREE = 1;

    // Release the SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}


/************************************************************************************//**
 *
 * @brief InitSpiGpioADC()
 *          Initialize SPIA GPIOs
 *          GPIO16 - GPIO19 as SPISIMOA, SPISOMIA, SPICLKA, SPISTEA.
 *          Pullup enabled and QSEL async.
 *          SPISTEA stands for Chip Select. See connections in header file.
 *
 * @param[in]   -
 *
 * @return      None
 *
 */
void InitSpiGpioADC(void)
{
    EALLOW;
    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;  // Enable pull-up on GPIO58 (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;  // Enable pull-up on GPIO59 (SPISOMIA)
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;  // Enable pull-up on GPIO60 (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;  // Enable pull-up on GPIO61 (SPISTEA)

    //
    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    //
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO58 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Asynch input GPIO59 (SPISOMIA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO60 (SPICLKA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3; // Asynch input GPIO61 (SPISTEA)

    //
    // Configure SPI-A pins using GPIO registers
    //
    // Write the appropriate values to GPyMUX1/2 and GPyGMUX1/2.
    // These were defined using table 8-7 pages 959-963 on technical reference manual
    // "When changing the GPyGMUX value for a pin, always set the corresponding GPyMUX
    // to zero first to avoid glitching in the muxes." (page 953)
    //
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;

    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 3;

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;

    EDIS;
}


//
//
//
/************************************************************************************//**
 *
 * @brief SetupGpioADC()
 *          Configure GPIO0 as a falling edge triggered interrupt for DRDY signal.
 *          This signal indicates availability of new conversion data.
 *
 * @param[in]   -
 *
 * @return      None
 *
 */
void SetupGpioADC(void){
    // 1. Setup GPIO for DRDY pin (external interrupt)
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;          // input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO0 = 0;        // XINT1 Synch to SYSCLKOUT only
    EDIS;

    //
    // GPIO0 is XINT1
    //
    GPIO_SetupXINT1Gpio(0);

    //
    // Configure XINT1
    //
    XintRegs.XINT1CR.bit.POLARITY = 0;          // Falling edge interrupt
    XintRegs.XINT2CR.bit.POLARITY = 1;          // Rising edge interrupt

    //
    // Enable XINT1 and XINT2
    //
    XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1


    // 2. Setup GPIO for ADC /RESET pin, set it high
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    // 3. Setup GPIO for ADC START pin, set it low;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    EDIS;

}

/************************************************************************************//**
 *
 * @brief
 *
 *
 * @param[in]   -
 *
 * @return      None
 *
 */



/************************************************************************************//**
 *
 *
 * @brief InitADCPeripherals()
 *          Initialize MCU peripherals and pins to interface with ADC
 *
 * @param[in]   *adcChars  ADC characteristics
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 *
 * @code
 *     if ( !InitADCPeripherals( &adcChars, &spiHdl ) ) {
 *        // Error initializing MCU SPI Interface
 *        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
 *        return( false );
 *     }
 *     // MCU initialized ADC successfully
 * @endcode
 */
bool InitADCPeripherals( ADCchar_Set *adcChars)
{
    bool            status;

//    *spiHdl = SPI_open( ADC_SPI_0, &spiParams );
//    if (*spiHdl == NULL) {
//        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
//        while (1);
//    }
//    else {
//        Display_printf( displayHdl, 0, 0, "Master SPI initialized\n" );
//    }

    InitSpiADS124S08();     // Configure SPI interface (CPOL = 0, CPHA =1);

    InitSpiGpioADC();       // Initializes GPIO58 - GPIO61 as SPISIMOA, SPISOMIA, SPICLKA, SPISTEA.

    SetupDRDYGpio();        // Sets GPIO0 as falling edge external interruption

    // Start up the ADC
    status = adcStartupRoutine( adcChars, *spiHdl );

    /* DRDY interrupt configuration */
//    GPIO_clearInt( ADC_DRDY );
//    GPIO_enableInt( ADC_DRDY );                           // enable Interrupt

    startConversions( *spiHdl );                           // Start Conversions

    return( status );
}

/************************************************************************************//**
 *
 * @brief toggleRESET()
 *            Pulses the /RESET GPIO pin low
 *
 * @return      None
 */
void toggleRESET( void )
{
    EALLOW;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;

    // Minimum nRESET width: 4 tCLKs = 4 * 1/4.096MHz =
    DELAY_US( DELAY_4TCLK );

    GpioDataRegs.GPASET.bit.GPIO1 = 1;
}

