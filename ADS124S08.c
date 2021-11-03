//###########################################################################
//
// FILE:   ADS124S08.c
//
// TITLE:  Functions to support ADS124S08EVM using launchpad F28379D
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
/*
 * ADS124S08.c
 *
 *  Created on: 30 de set de 2021
 *      Author: Lucas Bertolossi
 */

/*
 * Software to use ADS124S08EVM with F28379D
 *
 */

#include <stdint.h>
#include "F28x_Project.h"
#include <math.h>
#include "ADS124S08.h"
/* RTD related includes */
#include "inc/rtd.h"
#include "inc/rtd_tables.h"
//#include "device.h"
typedef enum RTDExampleDef{
    RTD_2_Wire_Fig15,     // 2-Wire RTD example using ADS124S08 EVM, User's Guide Figure 15
    RTD_3_Wire_Fig14,     // 3-Wire RTD example using ADS124S08 EVM, User's Guide Figure 14
    RTD_4_Wire_Fig16,     // 4-Wire RTD example using ADS124S08 EVM, User's Guide Figure 16
} RTD_Example;

#define td_CSSC 1000000   // Delay time, first SCLK rising edge after CS falling edge (min 20 ns, no maximum value)

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/* Internal register map array (to recall current configuration) */
static uint16_t registerMap[NUM_REGISTERS];

//****************************************************************************
//
// Functions
//
//****************************************************************************

// InitSPIADS124S08 - This function initializes the SPI to a known state to work with ADS124S08EVM
//
void InitSpiADS124S08(void)
{
    // Step 1. Clear the SPI Software Reset bit (SPISWRESET) to 0 to force the SPI to the reset state.
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    // Step 2. Configure the SPI as desired:
    // � Select either master or slave mode (MASTER_SLAVE).
    //   Enable master (0 == slave, 1 == master)
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;

    // � Choose SPICLK polarity and phase (CLKPOLARITY and CLK_PHASE). SPI mode 1 selected.
    //   Clock polarity (0 == rising, 1 == falling), ADS124S08 operates in SPI mode 1 (CPOL = 0, CPHA = 1)
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    //   Clock phase (0 == normal, 1 == delayed)
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

    // � Set the desired baud rate (SPIBRR).
    //   ADS124S08 - Minimum period for SCLK = 100 ns (maximum frequency is 10 MHz)
    //   Baud rate for HS_MODE = 0 is LSPCLK/(SPIBRR+1) (example 18-3)
    //   Default SYSCLK = 200 MHz
    //   LSPCLKDIV is set to default /4 table 3-164 page 345 (LOSPCP register field)
    //   LSPCLK = 200 / 4 = 50 MHz, SPIBRR = 49 was chosen -> baud rate = 1 MHz
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR_ADS124S08;

    // � Enable high speed mode if desired (HS_MODE).  (Not needed)
    // � Set the SPI character length (SPICHAR) to 8.
    SpiaRegs.SPICCR.bit.SPICHAR = (SPI_WORD_SIZE-1);

    // � Clear the SPI Flags (OVERRUN_FLAG, INT_FLAG). Cleared setting SPISWRESET to 0
//    SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;

    // � Enable SPISTE inversion (STEINV) if needed. (Not needed)
    // � Enable 3-wire mode (TRIWIRE) if needed. (Not needed)

    // � If using FIFO enhancements:
    // � Enable the FIFO enhancements (SPIFFENA). > SPIFFTX.14 = 1
    // � Clear the FIFO Flags (TXFFINTCLR, RXFFOVFCLR, and RXFFINTCLR).
    //   SPIFFTX.6 = 1 / SPIFFRX.14 = 1 / SPIFFRX.6 = 1
    // � Release transmit and receive FIFO resets (TXFIFO and RXFIFORESET).
    //   SPIFFTX.13 = 1 / SPIFFRX.13 = 1
    // � Release SPI FIFO channels from reset (SPIRST).
    //   SPIFFTX.15 = 1
    // Step 3. If interrupts are used:
    // � In non-FIFO mode, enable the receiver overrun and/or SPI interrupts (OVERRUNINTENA
    // and SPIINTENA).
    // � In FIFO mode, set the transmit and receive interrupt levels (TXFFIL and RXFFIL) then
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


//
// Initialize SPIA GPIOs
// GPIO16 - GPIO19 as SPISIMOA, SPISOMIA, SPICLKA, SPISTEA.
// Pullup enabled and QSEL async.
// SPISTEA stands for Chip Select. See connections in header file.
//
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
// Configure GPIO0 as a falling edge triggered interrupt for DRDY signal.
// This signal indicates availability of new conversion data.
//
void SetupDRDYGpio(void){
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

}



////
//// clearShipSelect - Sets CS Pin to low and delay for a minimum of td(CSSC)
////
//void clearChipSelect(void){
//    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
//    DELAY_US(td_CSSC);               // Delays for 1 microsecond (minimum is 20 ns)
//}
//
////
//// setChipSelect - Sets CS Pin to high. Performs no waiting.
////
//void setChipSelect(void){
//    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
//}

//
// spi_xmit - Transmit 8 bits (LEFT-JUSTIFIED). Example: send (08h) -> spi_send(0x0800)
// Data written to SPIDAT or SPITXBUF initiates data transmission on the SPISIMO pin,
// MSB (most significant bit) first.
void spi_send(uint16_t data)
{
//    data = data << 8;
    SpiaRegs.SPITXBUF = data;
}

/*
 * Writes a single of register with the specified data
 *
 * \param regnum addr_mask 8-bit mask of the register to which we start writing
 * \param data to be written
 *
 */

//
// spi_xmit - Transmit value via SPI
//
uint16_t spi_xmit(Uint16 a)
{
    uint16_t rx;
    SpiaRegs.SPITXBUF = a;
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }   // mayybe needs to change to interrupt depending on adc returning value or not
    rx = SpiaRegs.SPIRXBUF;
    return rx;
}

uint16_t regWrite(uint16_t regnum, uint16_t data)
{
    uint16_t iDataTx[3];
    uint16_t iDataRx[3];
    int i;
    iDataTx[0] = (OPCODE_WREG + (regnum & 0x1f)) << 8;
    iDataTx[1] = 0x0000;
    iDataTx[2] = data << 8;
//    clearChipSelect();
    for (i = 0; i < 3; i++){
        iDataRx[i] = spi_xmit(iDataTx[i]);
//        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }   // mayybe needs to change to interrupt depending on adc returning value or not
//        iDataRx[i] = SpiaRegs.SPIRXBUF;
        }
//    setChipSelect();
    return iDataRx[2];    // doesn't need to return, but im testing loopback
}

/*
 * Reads a single register contents from the specified address
 *
 * \param regnum identifies which address to read
 *
 */
uint16_t regRead(uint16_t regnum)
{
    int i;
    uint16_t iDataTx[3];
    uint16_t iDataRx[3];
//    uint16_t junk;
    iDataTx[0] = (OPCODE_RREG + (regnum & 0x1f)) << 8;
    iDataTx[1] = 0x00;
    iDataTx[2] = 0x00;    // just clock data into RX buffer
//    clearChipSelect();
//    /* MUST MUST MUST purge junk from fifo!!!! */
//    while(SSIDataGetNonBlocking(SPI_BASE, &junk));
    for(i = 0; i < 3; i++){
        iDataRx[i] = spi_xmit(iDataTx[i]);
        }
//    if(regnum < NUM_REGISTERS)
//            registers[regnum] = iDataRx[2];
//    setChipSelect();
    return iDataRx[2];
}
////
//// Read the last conversion result
////
//int dataRead(uint32_t *dStatus, uint32_t *dData, uint32_t *dCRC)
//{
//    uint32_t xcrc;
//    uint32_t xstatus;
//    int iData;
//    clearChipSelect();
//    if((registers[SYS_ADDR_MASK] & 0x01) == DATA_MODE_STATUS)
//    {
//        xstatus = xferWord(0x00);
//        dStatus[0] = (uint8_t)xstatus;
//    }
//
//    // get the conversion data
//    iData = xferWord(0x00);
//    iData = (iData<<8) + xferWord(0x00);
//#ifdef ADS124S08
//    iData = (iData<<8) + xferWord(0x00);
//#endif
//    if((registers[SYS_ADDR_MASK] & 0x02) == DATA_MODE_CRC)
//    {
//        xcrc = xferWord(0x00);
//        dCRC[0] = (uint8_t)xcrc;
//    }
//    setChipSelect();
//    return iData ;
//}

void readRTDtemp(void){
    RTD_Set     *rtdSet = NULL;
    RTD_Type    rtdType = Pt;
    RTD_Example rtdExample = RTD_4_Wire_Fig16;
    float       rtdRes, rtdTemp;
    ADCchar_Set adcChars, adcChars2;
    uint16_t     status;



    switch ( rtdType ) {
        case Pt:
            rtdSet = &PT100_RTD;
            break;
    }

    switch ( rtdExample ) {
//        case RTD_2_Wire_Fig15:
//            adcChars.inputMuxConfReg = RTD_TWO_WIRE_INPUT_MUX;
//            adcChars.pgaReg          = RTD_TWO_WIRE_PGA;
//            adcChars.dataRateReg     = RTD_TWO_WIRE_DATARATE;
//            adcChars.refSelReg       = RTD_TWO_WIRE_REF_SEL;
//            adcChars.IDACmagReg      = RTD_TWO_WIRE_IDACMAG;
//            adcChars.IDACmuxReg      = RTD_TWO_WIRE_IDACMUX;
//            adcChars.Vref            = RTD_TWO_WIRE_EXT_VREF;
//            rtdSet->Rref             = RTD_TWO_WIRE_REF_RES;
//            rtdSet->wiring           = Two_Wire_High_Side_Ref;
//            break;
//        case RTD_3_Wire_Fig14:
//            adcChars.inputMuxConfReg = RTD_THREE_WIRE_INPUT_MUX;
//            adcChars.pgaReg          = RTD_THREE_WIRE_PGA;
//            adcChars.dataRateReg     = RTD_THREE_WIRE_DATARATE;
//            adcChars.refSelReg       = RTD_THREE_WIRE_REF_SEL;
//            adcChars.IDACmagReg      = RTD_THREE_WIRE_IDACMAG;
//            adcChars.IDACmuxReg      = RTD_THREE_WIRE_IDACMUX;
//            adcChars.Vref            = RTD_THREE_WIRE_EXT_VREF;
//            rtdSet->Rref             = RTD_THREE_WIRE_REF_RES;
//            rtdSet->wiring           = Three_Wire_High_Side_Ref_Two_IDAC;
//            break;
        case RTD_4_Wire_Fig16:
            adcChars.inputMuxConfReg = RTD_FOUR_WIRE_INPUT_MUX;
            adcChars.pgaReg          = RTD_FOUR_WIRE_PGA;
            adcChars.dataRateReg     = RTD_FOUR_WIRE_DATARATE;
            adcChars.refSelReg       = RTD_FOUR_WIRE_REF_SEL;
            adcChars.IDACmagReg      = RTD_FOUR_WIRE_IDACMAG;
            adcChars.IDACmuxReg      = RTD_FOUR_WIRE_IDACMUX;
            adcChars.Vref            = RTD_FOUR_WIRE_INT_VREF;
            rtdSet->Rref             = RTD_FOUR_WIRE_REF_RES;
            rtdSet->wiring           = Four_Wire_High_Side_Ref;
            break;
    }
    adcChars.VBIASReg = RTD_VBIAS;

    /* add in device initialization */
    if ( !InitADCPeripherals(&adcChars) ) {
//        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
        while (1);
    }

    do {
        if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
            adcChars.adcValue1 = readConvertedData( spiHdl, &status, COMMAND );
            // For 3-wire with one IDAC, multiple readings are needed. So reconfigure ADC to read 2nd channel
//            if ( rtdSet->wiring == Three_Wire_High_Side_Ref_One_IDAC || rtdSet->wiring == Three_Wire_Low_Side_Ref_One_IDAC ) {
//                adcChars2.inputMuxConfReg = RTD_THREE_WIRE_INPUT_MUX2;
//                adcChars2.pgaReg          = RTD_THREE_WIRE_PGA;
//                adcChars2.dataRateReg     = RTD_THREE_WIRE_DATARATE;
//                adcChars2.refSelReg       = RTD_THREE_WIRE_REF_SEL;
//                adcChars2.IDACmagReg      = RTD_THREE_WIRE_IDACMAG;
//                adcChars2.IDACmuxReg      = RTD_THREE_WIRE_IDACMUX;
//                adcChars2.Vref            = RTD_THREE_WIRE_EXT_VREF;
//                adcChars2.VBIASReg        = VBIAS_DEFAULT;
//
//
//                if ( !ReconfigureADC( &adcChars2, spiHdl ) ) {
//                    Display_printf( displayHdl, 0, 0, "Error reconfiguring ADC\n" );
//                    while (1);
//                }
//                // Store second channel value into previous ADC structure as adcValue2
//                if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
//                   adcChars.adcValue2 = readConvertedData( spiHdl, &status, COMMAND );
//                } else {
//                    Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
//                    while (1);
//                }
//            }
            // Convert ADC values RTD resistance
            rtdRes = Convert_Code2RTD_Resistance( &adcChars, rtdSet  );
            // Convert RTD resistance to temperature and linearize
            rtdTemp = RTD_Linearization( rtdSet, rtdRes );
//            Display_printf( displayHdl, 0, 0, "ADC conversion result 1: %i\n", adcChars.adcValue1 );
//            if ( rtdSet->wiring == Three_Wire_High_Side_Ref_One_IDAC || rtdSet->wiring == Three_Wire_Low_Side_Ref_One_IDAC ) {
//                Display_printf( displayHdl, 0, 0, "ADC conversion result 2: %i\n", adcChars.adcValue2 );
//            }
            if ( isnan(rtdTemp) ) {
                Display_printf( displayHdl, 0, 0, "RTD temperature: NaN \n\n" );
            } else {
                Display_printf( displayHdl, 0, 0, "RTD temperature: %.3f (C)\n\n", rtdTemp );
            }

        } else {
//            Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
            while (1);
        }

    } while (1);

}

/************************************************************************************//**
 *
 * @brief adcStartupRoutine()
 *          Startup function to be called before communicating with the ADC
 *
 * @param[in]   *adcChars  ADC characteristics
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 */
bool adcStartupRoutine(ADCchar_Set *adcChars)
{
    uint16_t initRegisterMap[NUM_REGISTERS] = { 0 };   // adapted from 8 bits to 16
    uint16_t status, i;                                // adapted from 8 bits to 16
    // Provide additional delay time for power supply settling
    DELAY_US( DELAY_2p2MS );

    // Toggle nRESET pin to assure default register settings.
    toggleRESET();
    // Must wait 4096 tCLK after reset
    usleep( DELAY_4096TCLK );

    status = regRead(REG_ADDR_STATUS);
    if ( (status & ADS_nRDY_MASK) ) {
        return( false );                      // Device not ready
    }

    // Ensure internal register array is initialized
    restoreRegisterDefaults();

    // Configure initial device register settings here
    initRegisterMap[REG_ADDR_STATUS]  = 0x00;                      // Reset POR event
    initRegisterMap[REG_ADDR_INPMUX]  = adcChars->inputMuxConfReg;
    initRegisterMap[REG_ADDR_PGA]     = adcChars->pgaReg;
    initRegisterMap[REG_ADDR_DATARATE]= adcChars->dataRateReg;
    initRegisterMap[REG_ADDR_REF]     = adcChars->refSelReg;
    initRegisterMap[REG_ADDR_IDACMAG] = adcChars->IDACmagReg;
    initRegisterMap[REG_ADDR_IDACMUX] = adcChars->IDACmuxReg;
    initRegisterMap[REG_ADDR_VBIAS]   = adcChars->VBIASReg;
    initRegisterMap[REG_ADDR_SYS]     = SYS_DEFAULT;

    // Initialize ADC Characteristics
    adcChars->resolution     = ADS124S08_BITRES;
    adcChars->VBIASReg       = VBIAS_DEFAULT;
    adcChars->offsetCalCoeff = 0;
    adcChars->gainCalCoeff   = 1;
    adcChars->sampleRate     = 20;      // 20 samples per second
    adcChars->pgaGain        = pow(2, (adcChars->pgaReg & ADS_GAIN_MASK) );

    // Write to all modified registers
    writeMultipleRegisters( spiHdl, REG_ADDR_STATUS, REG_ADDR_SYS - REG_ADDR_STATUS + 1, initRegisterMap );

    // Read back all registers
    readMultipleRegisters( spiHdl, REG_ADDR_ID, NUM_REGISTERS );
    for ( i = REG_ADDR_STATUS; i < REG_ADDR_SYS - REG_ADDR_STATUS + 1; i++ ) {
        if ( i == REG_ADDR_STATUS )
            continue;
        if ( initRegisterMap[i] != registerMap[i] )
            return( false );
    }
    return( true );
}


 /************************************************************************************//**
  *
  * @brief restoreRegisterDefaults()
  *          Updates the registerMap[] array to its default values
  *          NOTES: If the MCU keeps a copy of the ADC register settings in memory,
  *          then it is important to ensure that these values remain in sync with the
  *          actual hardware settings. In order to help facilitate this, this function
  *          should be called after powering up or resetting the device (either by
  *          hardware pin control or SPI software command).
  *          Reading back all of the registers after resetting the device will
  *          accomplish the same result.
  *
  * @return      None
  */
void restoreRegisterDefaults( void )
{
     /* Default register settings */
     registerMap[REG_ADDR_ID]       = ID_DEFAULT;
     registerMap[REG_ADDR_STATUS]   = STATUS_DEFAULT;
     registerMap[REG_ADDR_INPMUX]   = INPMUX_DEFAULT;
     registerMap[REG_ADDR_PGA]      = PGA_DEFAULT;
     registerMap[REG_ADDR_DATARATE] = DATARATE_DEFAULT;
     registerMap[REG_ADDR_REF]      = REF_DEFAULT;
     registerMap[REG_ADDR_IDACMAG]  = IDACMAG_DEFAULT;
     registerMap[REG_ADDR_IDACMUX]  = IDACMUX_DEFAULT;
     registerMap[REG_ADDR_VBIAS]    = VBIAS_DEFAULT;
     registerMap[REG_ADDR_SYS]      = SYS_DEFAULT;
     registerMap[REG_ADDR_OFCAL0]   = OFCAL0_DEFAULT;
     registerMap[REG_ADDR_OFCAL1]   = OFCAL1_DEFAULT;
     registerMap[REG_ADDR_OFCAL2]   = OFCAL2_DEFAULT;
     registerMap[REG_ADDR_FSCAL0]   = FSCAL0_DEFAULT;
     registerMap[REG_ADDR_FSCAL1]   = FSCAL1_DEFAULT;
     registerMap[REG_ADDR_FSCAL2]   = FSCAL2_DEFAULT;
     registerMap[REG_ADDR_GPIODAT]  = GPIODAT_DEFAULT;
     registerMap[REG_ADDR_GPIOCON]  = GPIOCON_DEFAULT;
}

