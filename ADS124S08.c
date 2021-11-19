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
#include "adchal_tidrivers_adapted.h"
#include "F28379D_lcd.h"
#include <stdio.h>

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

/************************************************************************************//**
 *
 * @brief getRegisterValue()
 *          Getter function to access the registerMap array outside of this module
 *
 * @param[in]   address The 8-bit register address
 *
 * @return      The 8-bit register value
 */
uint16_t getRegisterValue( uint16_t address )
{
    assert( address < NUM_REGISTERS );
    return registerMap[address];
}


//// Comment if using SPISTEA and not GPIO /CS
//// clearShipSelect - Sets CS Pin to low and delay for a minimum of td(CSSC)
////
void clearChipSelect(void){
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    DELAY_US(td_CSSC);               // Delays for 1 microsecond (minimum is 20 ns)
}
//
////
//// setChipSelect - Sets CS Pin to high. Performs no waiting.
////
void setChipSelect(void){
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
}


//void spi_send(uint16_t data)
//{
////    data = data << 8;
//    SpiaRegs.SPITXBUF = data;
//}



//uint16_t regWrite(uint16_t regnum, uint16_t data)
void regWrite(uint16_t regnum, uint16_t data)
{
    uint16_t iDataTx[3];
    uint16_t iDataRx[3] = { 0xffff, 0xffff, 0xffff };
    uint16_t i;
    uint16_t junk;
    iDataTx[0] = (0x0000 | (OPCODE_WREG | (regnum & 0x1f)));
    iDataTx[1] = 0x0000;
    iDataTx[2] = data;
    clearChipSelect();
//    junk = SpiaRegs.SPIRXBUF;
    xferWord(iDataTx[0]);
    xferWord(iDataTx[1]);
    xferWord(iDataTx[2]);
//    for (i = 0; i < 3; i++){
//        iDataRx[i] = spi_xmit(iDataTx[i]);
//        }
//    for(i = 0; i < 3; i++){
//        SpiaRegs.SPITXBUF = (iDataTx[i] << 8);
//        while(SpiaRegs.SPIFFRX.bit.RXFFINT !=1) { };
//        iDataRx[i] = SpiaRegs.SPIRXBUF;
//        SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
//    }
    setChipSelect();
    return;
}



/*
 * Reads a single register contents from the specified address
 *
 * \param regnum identifies which address to read
 *
 */
uint16_t regRead(uint16_t regnum)
{
    uint16_t iDataTx[3];
    uint16_t iDataRx[3] = {0xffff, 0xffff, 0xffff};
    uint16_t i;
    uint16_t junk;
    iDataTx[0] = (0x0000 | (OPCODE_RREG | (regnum & 0x1f)));
    iDataTx[1] = 0x00;
    iDataTx[2] = 0x00;    // clock data into TX buffer to read data

    clearChipSelect();
//    /* MUST MUST MUST purge junk from fifo!!!! */
//    while(SSIDataGetNonBlocking(SPI_BASE, &junk));
    while(SpiaRegs.SPIFFRX.bit.RXFFST != 0){
        junk = SpiaRegs.SPIRXBUF;
    }
    junk = SpiaRegs.SPIRXBUF;

    for(i = 0; i < 3; i++){
        SpiaRegs.SPITXBUF = (iDataTx[i] << 8);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }   // maybe needs to change to interrupt depending on adc returning value or not
        iDataRx[i] = SpiaRegs.SPIRXBUF;
        }
//    for(i = 0; i < 3; i++){
//        SpiaRegs.SPITXBUF = (iDataTx[i] << 8);
//        while(SpiaRegs.SPIFFRX.bit.RXFFINT !=1) { };
//        iDataRx[i] = SpiaRegs.SPIRXBUF;
//        SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
//    }
    setChipSelect();
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
    ADCchar_Set adcChars;
    uint16_t     status;
//    char cRtdTemp[16] = {0};
//    char errorTimeOut[] = "Timeout on conv.";
    char errorSpiConfig[] = "Error in SPI";


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

    if ( !InitADCPeripherals(&adcChars) ) {
        DisplayLCD(1, errorSpiConfig);
        DisplayLCD(2, "");

        while (1);
    }

    do {
        if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
            adcChars.adcValue1 = readConvertedData( &status, COMMAND );
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
//            sprintf(cRtdTemp, "%s %.3f C","Temp:", rtdTemp);
            if ( isnan(rtdTemp) ) {
                DisplayLCD(1, "Temp: NaN");
                DisplayLCD(2, "");
            } else {
//                DisplayLCD(1, cRtdTemp);
            }

        } else {
//            DisplayLCD(1, errorTimeOut);
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
    uint16_t status;                                // adapted from 8 bits to 16
    uint16_t i = 0;
//    uint16_t status_a = 0x4444;
//    uint16_t status_b = 0x4444;
//    uint16_t status_c = 0x4444;
//    uint16_t status_d = 0x4444;
//    uint16_t status_e = 0x4444;
//    uint16_t status_f = 0x4444;
//    uint16_t status_g = 0x4444;
//    uint16_t status_h = 0x4444;
//    uint16_t status_i = 0x4444;
//    uint16_t inpmux = 0x4444;
    uint16_t j = 0;



    // Provide additional delay time for power supply settling
    DELAY_US( DELAY_2p2MS );

    clearChipSelect();
    // Toggle nRESET pin to assure default register settings.
    toggleRESET();
    // Must wait 4096 tCLK after reset
    DELAY_US( DELAY_4096TCLK );
    setChipSelect();
// TEST

    // Ensure internal register array is initialized
    restoreRegisterDefaults();

    // Read back all registers
//    readMultipleRegisters( spiHdl, REG_ADDR_ID, NUM_REGISTERS );
    registerMap[REG_ADDR_ID] = regRead(REG_ADDR_ID);
    registerMap[REG_ADDR_STATUS] = regRead(REG_ADDR_STATUS);
    registerMap[REG_ADDR_INPMUX] = regRead(REG_ADDR_INPMUX);
    registerMap[REG_ADDR_PGA] = regRead(REG_ADDR_PGA);
    registerMap[REG_ADDR_DATARATE] = regRead(REG_ADDR_DATARATE);
    registerMap[REG_ADDR_REF] = regRead(REG_ADDR_REF);
    registerMap[REG_ADDR_IDACMAG] = regRead(REG_ADDR_IDACMAG);
    registerMap[REG_ADDR_IDACMUX] = regRead(REG_ADDR_IDACMUX);
    registerMap[REG_ADDR_VBIAS] = regRead(REG_ADDR_VBIAS);
    registerMap[REG_ADDR_SYS] = regRead(REG_ADDR_SYS);

// TEST
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
//    writeMultipleRegisters( spiHdl, REG_ADDR_STATUS, REG_ADDR_SYS - REG_ADDR_STATUS + 1, initRegisterMap );
    // Can improve this following part in the future
    regWrite(REG_ADDR_ID, 0x00);
    regWrite(REG_ADDR_STATUS, 0x00);
    regWrite(REG_ADDR_INPMUX, adcChars->inputMuxConfReg);
    regWrite(REG_ADDR_PGA, adcChars->pgaReg);
    regWrite(REG_ADDR_DATARATE, adcChars->dataRateReg);
    regWrite(REG_ADDR_REF, adcChars->refSelReg);
    regWrite(REG_ADDR_IDACMAG, adcChars->IDACmagReg);
    regWrite(REG_ADDR_IDACMUX, adcChars->IDACmuxReg);
    regWrite(REG_ADDR_VBIAS, adcChars->VBIASReg);
    regWrite(REG_ADDR_SYS, SYS_DEFAULT);

    DELAY_US(10);
//    regWrite(REG_ADDR_ID, 0x00);
//    regWrite(REG_ADDR_STATUS, 0x00);
//    regWrite(REG_ADDR_INPMUX, adcChars->inputMuxConfReg);
//    regWrite(REG_ADDR_PGA, adcChars->pgaReg);
//    regWrite(REG_ADDR_DATARATE, adcChars->dataRateReg);
//    regWrite(REG_ADDR_REF, adcChars->refSelReg);
//    regWrite(REG_ADDR_IDACMAG, adcChars->IDACmagReg);
//    regWrite(REG_ADDR_IDACMUX, adcChars->IDACmuxReg);
//    regWrite(REG_ADDR_VBIAS, adcChars->VBIASReg);
//    regWrite(REG_ADDR_SYS, SYS_DEFAULT);

//    setChipSelect();
//
//    clearChipSelect();

    // Read back all registers
//    readMultipleRegisters( spiHdl, REG_ADDR_ID, NUM_REGISTERS );
    registerMap[REG_ADDR_STATUS] = regRead(REG_ADDR_STATUS);
    registerMap[REG_ADDR_INPMUX] = regRead(REG_ADDR_INPMUX);
    registerMap[REG_ADDR_PGA] = regRead(REG_ADDR_PGA);
    registerMap[REG_ADDR_DATARATE] = regRead(REG_ADDR_DATARATE);
    registerMap[REG_ADDR_REF] = regRead(REG_ADDR_REF);
    registerMap[REG_ADDR_IDACMAG] = regRead(REG_ADDR_IDACMAG);
    registerMap[REG_ADDR_IDACMUX] = regRead(REG_ADDR_IDACMUX);
    registerMap[REG_ADDR_VBIAS] = regRead(REG_ADDR_VBIAS);
    registerMap[REG_ADDR_SYS] = regRead(REG_ADDR_SYS);



//    while(1){}
    // Check if all registers were written correctly
    for ( i = REG_ADDR_STATUS; i < REG_ADDR_SYS - REG_ADDR_STATUS + 1; i++ ) {
        if ( i == REG_ADDR_STATUS )         // ignores status register in case POR flag is set
            continue;
        if ( initRegisterMap[i] != registerMap[i] )
            return( false );
    }
    return( true );
}


 /************************************************************************************//**
  *
  * @brief restoreRegisterDefaults()
  *          Updates the registerMap[]  array to its default values
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
void restoreRegisterDefaults(void)
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


/************************************************************************************//**
 *
 * @brief startConversions()
 *          Wakes the device from power-down and starts continuous conversions
 *            by setting START pin high or sending START Command
 *
 * @param[in]
 *
 * @return      None
 */
void startConversions(void)
{
    // Wakeup device
    sendWakeup();

#ifdef START_PIN_CONTROLLED
     /* Begin continuous conversions */
    setSTART( HIGH );
#else
    sendSTART( spiHdl );
#endif
}


/************************************************************************************//**
 *
 * @brief stopConversions()
 *          Stops continuous conversions by setting START pin low or sending STOP Command
 *
 * @param[in]
 *
 * @return      None
 */
void stopConversions()
{
     /* Stop continuous conversions */
#ifdef START_PIN_CONTROLLED
    setSTART(LOW);
#else
    sendSTOP();
#endif
}


/************************************************************************************//**
 *
 * @brief readData()
 *          Sends the read command and retrieves STATUS (if enabled) and data
 *          NOTE: Call this function after /DRDY goes low and specify the
 *          the number of bytes to read and the starting position of data
 *
 * @param[in]   status[]    Pointer to location where STATUS byte will be stored
 * @param[in]   mode        Direct or Command read mode
 *
 * @return      32-bit sign-extended conversion result (data only)
 */
int32_t readConvertedData(uint16_t status[], readMode mode )
{
    uint16_t DataTx[RDATA_COMMAND_LENGTH + STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH] = { 0 };    // Initialize all array elements to 0
    uint16_t DataRx[RDATA_COMMAND_LENGTH + STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH] = { 0 };
    uint16_t byteLength;
    uint16_t dataPosition;
    uint16_t byte_options;
    bool    status_byte_enabled = 0;
    int32_t signByte, upperByte, middleByte, lowerByte;
    uint16_t i;
    // Status Byte is sent if SENDSTAT bit of SYS register is set
    byte_options = IS_SENDSTAT_SET << 1 | IS_CRC_SET;
    switch ( byte_options ) {
        case 0:                         // No STATUS and no CRC
            byteLength   = DATA_LENGTH;
            dataPosition = 0;
            break;
        case 1:                         // No STATUS and CRC
            byteLength   = DATA_LENGTH + CRC_LENGTH;
            dataPosition = 0;
            break;
        case 2:                         // STATUS and no CRC
            byteLength   = STATUS_LENGTH + DATA_LENGTH;
            dataPosition = 1;
            status_byte_enabled = 1;
            break;
        case 3:                         // STATUS and CRC
            byteLength   = STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH;
            dataPosition = 1;
            status_byte_enabled = 1;
            break;
    }

    if ( mode == COMMAND ) {
        DataTx[0]     = OPCODE_RDATA;
        byteLength   += 1;
        dataPosition += 1;
    }
//    spiSendReceiveArrays( spiHdl, DataTx, DataRx, byteLength );
    for(i = 0; i < byteLength; i++){
        DataRx[i] = spi_xmit(DataTx[i]);
    }


    // Parse returned SPI data
    /* Check if STATUS byte is enabled and if we have a valid "status" memory pointer */
    if ( status_byte_enabled && status ) {
        status[0] = DataRx[dataPosition - 1];
    }

    /* Return the 32-bit sign-extended conversion result */
    if ( DataRx[dataPosition] & 0x80u ) {
        signByte = 0xFF000000;
    } else {
        signByte = 0x00000000;
    }

    upperByte   = ((int32_t) DataRx[dataPosition] & 0xFF) << 16;
    middleByte  = ((int32_t) DataRx[dataPosition + 1] & 0xFF) << 8;
    lowerByte   = ((int32_t) DataRx[dataPosition + 2] & 0xFF);

    return ( signByte + upperByte + middleByte + lowerByte );
}


