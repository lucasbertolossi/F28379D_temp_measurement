//###########################################################################
//
// FILE:   ADS124S08.h
//
// TITLE:  Functions to support ADS124S08EVM using launchpad F28379D (header file)
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

/* Start definitions */
#ifndef ADS124S08_H_
#define ADS124S08_H_
#include "F28x_Project.h"
#include <stdint.h>

/********************************************************************************//**
*                               Hardware connections
*          F28379D Pin                 <==>         ADS124S08EVM Pin (J3)
*
*          SPISIMOA/GPIO58/J2Pin15     <==>         SDI
*          SPISOMIA/GPIO59/J2Pin14     <==>         SDO
*          SPICLKA/GPIO60/J1Pin7       <==>         SCLK
*          SPISTEA/GPIO61/J2Pin19      <==>         CS
*          GPIOXINT/GPIO0/J4Pin40      <==>         DRDY
*                   GPIO1/J4Pin39      <==>         _RESET
*                   GPIO2/J4Pin38      <==>         START
*
*********************************************************************************//**
 *                              ADS124S08EVM headers
 *
 * @name Board Resistance
 */
#define R68 1000    // Biasing resistors
#define R70 1000
#define R70 1000

/********************************************************************************//**
 *
 * @name START Command instead of START pin
 */
#define START_PIN_CONTROLLED    // Use START_PIN instead of START and STOP commands

/********************************************************************************//**
 *
 * @name RESET Command instead of RESET pin
 */
#define RESET_PIN_CONTROLLED    // Use RESET pin

/********************************************************************************//**
 *  *
 * @name RTD Connections for 4-Wire (Figure 16 of ADS124S08 EVM User's Guide) on J7
 *
 * AIN2 <- AINP
 * AIN4 <- AINN
 * AIN5 -> IDAC1
 * AIN6 <- REFP1
 * AIN7 <- REFN1
 */
#define RTD_VBIAS               VBIAS_DEFAULT
#define RTD_FOUR_WIRE_INPUT_MUX ADS_P_AIN2 | ADS_N_AIN4;                                    // MuxP = Ain2, MuxN = Ain4
#define RTD_FOUR_WIRE_PGA       ADS_PGA_ENABLED | ADS_GAIN_2;                               // PGA enabled, PGA gain = 2X
#define RTD_FOUR_WIRE_DATARATE  ADS_CONVMODE_CONT | ADS_FILTERTYPE_LL | ADS_DR_20;          // Continuous conversion, low latency filter, 20 SPS rate
#define RTD_FOUR_WIRE_REF_SEL   ADS_REFN_BYP_ENABLE | ADS_REFSEL_P1 | ADS_REFINT_ON_ALWAYS; // RefN Enabled, RefP1 and RefN1 selected, Int Ref always on
#define RTD_FOUR_WIRE_IDACMAG   ADS_IDACMAG_1000;                                           // IDAC Mag 1 mA
#define RTD_FOUR_WIRE_IDACMUX   ADS_IDAC2_OFF | ADS_IDAC1_A5;                               // IDAC Mux = AIN5, IDAC2 off
#define RTD_FOUR_WIRE_REF_RES   R68
#define RTD_FOUR_WIRE_INT_VREF  INT_VREF

/********************************************************************************//**
 * @brief Describes the ADC Characteristics structure
 */
typedef struct ADCcharDef {
    uint16_t     resolution;        // ADC bit resolution (8, 10, 12, 14, 16, 18, 24, 32)
                                    // resolution adapted from 8 bits to 16
    uint32_t     inputMuxConfReg;   // ADC input mux register configuration
    uint32_t     pgaGain;           // ADC PGA linear gain setting (1x, 2x, 4x, 8x, 16x, etc.)
    uint32_t     pgaReg;            // ADC PGA register configuration
    uint32_t     offsetCalCoeff;    // ADC Offset Calibration Coefficient (ideal ADC has offsetCalCoeff = 0)
    uint32_t     gainCalCoeff;      // ADC Gain Calibration Coefficient (ideal ADC has gainCalCoeff = 1)
    uint32_t     sampleRate;        // ADC sample rate in samples/sec
    uint32_t     dataRateReg;       // ADC data rate register configuration
    float        Vref;              // ADC Voltage reference
    uint32_t     refSelReg;         // ADC Reference selection register configuration
    uint32_t     startupDel;        // ADC Startup Delay: Delay between powering up ADC channel and starting a conversion
    uint32_t     burnoutCur;        // ADC Burnout Current Source
    uint32_t     IDACmuxReg;        // ADC IDAC mux selection register configuration
    uint32_t     IDACmagReg;        // ADC IDAC magnitude register configuration
    uint32_t     VBIASReg;          // ADC VBIAS register configuration
    uint32_t     staleness;         // ADC staleness of last reading
    int32_t      adcValue1;         // ADC conversion result provided as sign extended 2's complement (first one)
    int32_t      adcValue2;         // ADC conversion result provided as sign extended 2's complement (second one)
} ADCchar_Set;
/********************************************************************************//**
 *
 * @name Constants for ADS124S08
 *
 */
#define NUM_REGISTERS       ((uint16_t) 18)     // adapted from 8 bits to 16

#define ADS124S08_FCLK      4096000
#define ADS124S08_BITRES    24

// Lengths of conversion data components
#define COMMAND_LENGTH          2
#define DATA_LENGTH             3
#define STATUS_LENGTH           1
#define CRC_LENGTH              1
#define RDATA_COMMAND_LENGTH    1

// Flag to signal that we are in the process of collecting data
extern bool converting;
#define DATA_MODE_NORMAL    0x00
#define DATA_MODE_STATUS    0x01
#define DATA_MODE_CRC       0x02

#define INT_VREF            2.5

#define SPI_WORD_SIZE       8
#define SPI_SPEED           5000000


#define DELAY_4TCLK     (uint32_t) (1) // 1usec ~= (4.0 /ADS124S08_FCLK)
#define DELAY_4096TCLK  (uint32_t) (4096.0 * 1000000 / ADS124S08_FCLK )         // After RESET
#define DELAY_2p2MS     (uint32_t) (0.0022 * 1000000 / ADS124S08_FCLK )         // After power-up
#define TIMEOUT_COUNTER 10000

// Set SPIBRR to 11 so that clock will meet SPI slave's requirements
// LSPCLKDIV = 4 (default)
//
#if CPU_FRQ_200MHZ
#define SPI_BRR_ADS124S08 49
#endif


/********************************************************************************//**
 *
 * @name Command byte definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                  COMMAND                                      |
 * ---------------------------------------------------------------------------------
 */

    // SPI Control Commands
    #define OPCODE_NOP                  ((uint16_t) 0x00)   // adapted from 8 bits to 16
    #define OPCODE_WAKEUP               ((uint16_t) 0x02)    // adapted from 8 bits to 16
    #define OPCODE_POWERDOWN            ((uint16_t) 0x04)    // adapted from 8 bits to 16
    #define OPCODE_RESET                ((uint16_t) 0x06)    // adapted from 8 bits to 16
    #define OPCODE_START                ((uint16_t) 0x08)    // adapted from 8 bits to 16
    #define OPCODE_STOP                 ((uint16_t) 0x0A)    // adapted from 8 bits to 16

    //SPI Calibration Commands
    #define OPCODE_SYOCAL               ((uint16_t) 0x16)    // adapted from 8 bits to 16
    #define OPCODE_SYGCAL               ((uint16_t) 0x17)    // adapted from 8 bits to 16
    #define OPCODE_SFOCAL               ((uint16_t) 0x19)    // adapted from 8 bits to 16

    // SPI Data Read Command
    #define OPCODE_RDATA                ((uint16_t) 0x12)    // adapted from 8 bits to 16

    // SPI Register Read and Write Commands
    #define OPCODE_RREG                 ((uint16_t) 0x20)    // adapted from 8 bits to 16
    #define OPCODE_WREG                 ((uint16_t) 0x40)    // adapted from 8 bits to 16
    #define OPCODE_RWREG_MASK           ((uint16_t) 0x1F)    // adapted from 8 bits to 16

    /* Read mode enum */
    typedef enum {
        DIRECT,
        COMMAND
    } readMode;

/********************************************************************************//**
 *
 * @name  Register definitions
 */

/* ADS124S08 Register 0x0 (ID) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                 RESERVED[4:0]                            |            DEV_ID[2:0]           |
 *-----------------------------------------------------------------------------------------------
 *
 */
    // ID register address
    #define REG_ADDR_ID             ((uint16_t) 0x00)   // adapted from 8 bits to 16

    /** ID default (reset) value */
    #define ID_DEFAULT              ((uint16_t) 0x00)   // adapted from 8 bits to 16

    // Define DEV_ID
    #define ADS_124S08              0x00
    #define ADS_124S06              0x01
    #define ADS_114S08              0x04
    #define ADS_114S06              0x05


/* ADS124S08 Register 0x1 (STATUS) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 *------------------------------------------------------------------------------------------------
 *|  FL_POR  |    nRDY   | FL_P_RAILP| FL_P_RAILN| FL_N_RAILP| FL_N_RAILN| FL_REF_L1 | FL_REF_L0 |
 *------------------------------------------------------------------------------------------------
 */
   /** STATUS register address */
    #define REG_ADDR_STATUS         ((uint16_t) 0x01)   // adapted from 8 bits to 16

    /** STATUS default (reset) value */
    #define STATUS_DEFAULT          ((uint16_t) 0x80)   // adapted from 8 bits to 16

    #define ADS_FL_POR_MASK         0x80
    #define ADS_nRDY_MASK           0x40
    #define ADS_FL_P_RAILP_MASK     0x20
    #define ADS_FL_P_RAILN_MASK     0x10
    #define ADS_FL_N_RAILP_MASK     0x08
    #define ADS_FL_N_RAILN_MASK     0x04
    #define ADS_FL_REF_L1_MASK      0x02
    #define ADS_FL_REF_L0_MASK      0x10


/* ADS124S08 Register 0x2 (INPMUX) Definition
 *|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *------------------------------------------------------------------------------------------------
 *|                 MUXP[3:0]                    |                  MUXN[3:0]                    |
 *------------------------------------------------------------------------------------------------
 */
    // INPMUX register address
    #define REG_ADDR_INPMUX         ((uint16_t) 0x02)   // adapted from 8 bits to 16

    /** INPMUX default (reset) value */
    #define INPMUX_DEFAULT          ((uint16_t) 0x01)   // adapted from 8 bits to 16

    // Define the ADC positive input channels (MUXP)
    #define ADS_P_AIN0              0x00
    #define ADS_P_AIN1              0x10
    #define ADS_P_AIN2              0x20
    #define ADS_P_AIN3              0x30
    #define ADS_P_AIN4              0x40
    #define ADS_P_AIN5              0x50
    #define ADS_P_AIN6              0x60
    #define ADS_P_AIN7              0x70
    #define ADS_P_AIN8              0x80
    #define ADS_P_AIN9              0x90
    #define ADS_P_AIN10             0xA0
    #define ADS_P_AIN11             0xB0
    #define ADS_P_AINCOM            0xC0

    // Define the ADC negative input channels (MUXN)
    #define ADS_N_AIN0              0x00
    #define ADS_N_AIN1              0x01
    #define ADS_N_AIN2              0x02
    #define ADS_N_AIN3              0x03
    #define ADS_N_AIN4              0x04
    #define ADS_N_AIN5              0x05
    #define ADS_N_AIN6              0x06
    #define ADS_N_AIN7              0x07
    #define ADS_N_AIN8              0x08
    #define ADS_N_AIN9              0x09
    #define ADS_N_AIN10             0x0A
    #define ADS_N_AIN11             0x0B
    #define ADS_N_AINCOM            0x0C


/* ADS124S08 Register 0x3 (PGA) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|         DELAY[2:0]               |      PGA_EN[1:0]      |              GAIN[2:0]           |
 *-----------------------------------------------------------------------------------------------
 */
    // PGA register address
    #define REG_ADDR_PGA            ((uint16_t) 0x03)   // adapted from 8 bits to 16

    /** PGA default (reset) value */
    #define PGA_DEFAULT             ((uint16_t) 0x00)   // adapted from 8 bits to 16

    // Define conversion delay in tmod clock periods
    #define ADS_DELAY_14            0x00
    #define ADS_DELAY_25            0x20
    #define ADS_DELAY_64            0x40
    #define ADS_DELAY_256           0x60
    #define ADS_DELAY_1024          0x80
    #define ADS_DELAY_2048          0xA0
    #define ADS_DELAY_4096          0xC0
    #define ADS_DELAY_1             0xE0

    // Define PGA control
    #define ADS_PGA_BYPASS          0x00
    #define ADS_PGA_ENABLED         0x08

    // Define Gain
    #define ADS_GAIN_1              0x00
    #define ADS_GAIN_2              0x01
    #define ADS_GAIN_4              0x02
    #define ADS_GAIN_8              0x03
    #define ADS_GAIN_16             0x04
    #define ADS_GAIN_32             0x05
    #define ADS_GAIN_64             0x06
    #define ADS_GAIN_128            0x07
    #define ADS_GAIN_MASK           0x07


/* ADS124S08 Register 0x4 (DATARATE) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|   G_CHOP  |    CLK    |    MODE   |   FILTER  |                   DR[3:0]                   |
 *-----------------------------------------------------------------------------------------------
 */
    // DATARATE register address
    #define REG_ADDR_DATARATE       ((uint16_t) 0x04)   // adapted from 8 bits to 16

    /** DATARATE default (reset) value */
    #define DATARATE_DEFAULT        ((uint16_t) 0x14)   // adapted from 8 bits to 16

    #define ADS_GLOBALCHOP          0x80
    #define ADS_CLKSEL_EXT          0x40
    #define ADS_CONVMODE_SS         0x20
    #define ADS_CONVMODE_CONT       0x00
    #define ADS_FILTERTYPE_LL       0x10

    // Define the data rate */
    #define ADS_DR_2_5              0x00
    #define ADS_DR_5                0x01
    #define ADS_DR_10               0x02
    #define ADS_DR_16               0x03
    #define ADS_DR_20               0x04
    #define ADS_DR_50               0x05
    #define ADS_DR_60               0x06
    #define ADS_DR_100              0x07
    #define ADS_DR_200              0x08
    #define ADS_DR_400              0x09
    #define ADS_DR_800              0x0A
    #define ADS_DR_1000             0x0B
    #define ADS_DR_2000             0x0C
    #define ADS_DR_4000             0x0D


/* ADS124S08 Register 0x5 (REF) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|   FL_REF_EN[1:0]     | nREFP_BUF | nREFN_BUF |      REFSEL[1:0]      |      REFCON[1:0]     |
 *-----------------------------------------------------------------------------------------------
 */
    // REF register address
    #define REG_ADDR_REF            ((uint16_t) 0x05)   // adapted from 8 bits to 16

    /** REF default (reset) value */
    #define REF_DEFAULT             ((uint16_t) 0x10)   // adapted from 8 bits to 16

    #define ADS_FLAG_REF_DISABLE    0x00
    #define ADS_FLAG_REF_EN_L0      0x40
    #define ADS_FLAG_REF_EN_BOTH    0x80
    #define ADS_FLAG_REF_EN_10M     0xC0
    #define ADS_REFP_BYP_DISABLE    0x20
    #define ADS_REFP_BYP_ENABLE     0x00
    #define ADS_REFN_BYP_DISABLE    0x10
    #define ADS_REFN_BYP_ENABLE     0x00
    #define ADS_REFSEL_P0           0x00
    #define ADS_REFSEL_P1           0x04
    #define ADS_REFSEL_INT          0x08
    #define ADS_REFINT_OFF          0x00
    #define ADS_REFINT_ON_PDWN      0x01
    #define ADS_REFINT_ON_ALWAYS    0x02


/* ADS124S08 Register 0x6 (IDACMAG) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|FL_RAIL_EN|    PSW    |     0     |      0    |                  IMAG[3:0]                   |
 *-----------------------------------------------------------------------------------------------
 */
    // IDACMAG register address
    #define REG_ADDR_IDACMAG        ((uint16_t) 0x06)   // adapted from 8 bits to 16

    /** IDACMAG default (reset) value */
    #define IDACMAG_DEFAULT         ((uint16_t) 0x00)   // adapted from 8 bits to 16

    #define ADS_FLAG_RAIL_ENABLE    0x80
    #define ADS_FLAG_RAIL_DISABLE   0x00
    #define ADS_PSW_OPEN            0x00
    #define ADS_PSW_CLOSED          0x40
    #define ADS_IDACMAG_OFF         0x00
    #define ADS_IDACMAG_10          0x01
    #define ADS_IDACMAG_50          0x02
    #define ADS_IDACMAG_100         0x03
    #define ADS_IDACMAG_250         0x04
    #define ADS_IDACMAG_500         0x05
    #define ADS_IDACMAG_750         0x06
    #define ADS_IDACMAG_1000        0x07
    #define ADS_IDACMAG_1500        0x08
    #define ADS_IDACMAG_2000        0x09


/* ADS124S08 Register 0x7 (IDACMUX) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                   I2MUX[3:0]                 |                   I1MUX[3:0]                 |
 *-----------------------------------------------------------------------------------------------
 */
    // IDACMUX register address
    #define REG_ADDR_IDACMUX        ((uint16_t) 0x07)   // adapted from 8 bits to 16

    /** IDACMUX default (reset) value */
    #define IDACMUX_DEFAULT         ((uint16_t) 0xFF)   // adapted from 8 bits to 16

    // Define IDAC2 Output
    #define ADS_IDAC2_A0            0x00
    #define ADS_IDAC2_A1            0x10
    #define ADS_IDAC2_A2            0x20
    #define ADS_IDAC2_A3            0x30
    #define ADS_IDAC2_A4            0x40
    #define ADS_IDAC2_A5            0x50
    #define ADS_IDAC2_A6            0x60
    #define ADS_IDAC2_A7            0x70
    #define ADS_IDAC2_A8            0x80
    #define ADS_IDAC2_A9            0x90
    #define ADS_IDAC2_A10           0xA0
    #define ADS_IDAC2_A11           0xB0
    #define ADS_IDAC2_AINCOM        0xC0
    #define ADS_IDAC2_OFF           0xF0

    // Define IDAC1 Output
    #define ADS_IDAC1_A0            0x00
    #define ADS_IDAC1_A1            0x01
    #define ADS_IDAC1_A2            0x02
    #define ADS_IDAC1_A3            0x03
    #define ADS_IDAC1_A4            0x04
    #define ADS_IDAC1_A5            0x05
    #define ADS_IDAC1_A6            0x06
    #define ADS_IDAC1_A7            0x07
    #define ADS_IDAC1_A8            0x08
    #define ADS_IDAC1_A9            0x09
    #define ADS_IDAC1_A10           0x0A
    #define ADS_IDAC1_A11           0x0B
    #define ADS_IDAC1_AINCOM        0x0C
    #define ADS_IDAC1_OFF           0x0F


/* ADS124S08 Register 0x8 (VBIAS) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *| VB_LEVEL |  VB_AINC  |  VB_AIN5  |  VB_AIN4  |  VB_AIN3  |  VB_AIN2  |  VB_AIN1  |  VB_AIN0 |
 *-----------------------------------------------------------------------------------------------
 */
    // VBIAS register address
    #define REG_ADDR_VBIAS          ((uint16_t) 0x08)   // adapted from 8 bits to 16

    /** VBIAS default (reset) value */
    #define VBIAS_DEFAULT           ((uint16_t) 0x00)   // adapted from 8 bits to 16

    #define ADS_VBIAS_LVL_DIV2      0x00
    #define ADS_VBIAS_LVL_DIV12     0x80

    // Define VBIAS here
    #define ADS_VB_AINC             0x40
    #define ADS_VB_AIN5             0x20
    #define ADS_VB_AIN4             0x10
    #define ADS_VB_AIN3             0x08
    #define ADS_VB_AIN2             0x04
    #define ADS_VB_AIN1             0x02
    #define ADS_VB_AIN0             0x01


/* ADS124S08 Register 0x9 (SYS) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|            SYS_MON[2:0]          |     CAL_SAMP[1:0]     |  TIMEOUT  |    CRC    | SENDSTAT |
 *-----------------------------------------------------------------------------------------------
 */
    // SYS register address
    #define REG_ADDR_SYS            ((uint16_t) 0x09)   // adapted from 8 bits to 16

    /** SYS default (reset) value */
    #define SYS_DEFAULT             ((uint16_t) 0x10)   // adapted from 8 bits to 16

    #define ADS_SYS_MON_OFF         0x00
    #define ADS_SYS_MON_SHORT       0x20
    #define ADS_SYS_MON_TEMP        0x40
    #define ADS_SYS_MON_ADIV4       0x60
    #define ADS_SYS_MON_DDIV4       0x80
    #define ADS_SYS_MON_BCS_2       0xA0
    #define ADS_SYS_MON_BCS_1       0xC0
    #define ADS_SYS_MON_BCS_10      0xE0
    #define ADS_CALSAMPLE_1         0x00
    #define ADS_CALSAMPLE_4         0x08
    #define ADS_CALSAMPLE_8         0x10
    #define ADS_CALSAMPLE_16        0x18
    #define ADS_TIMEOUT_DISABLE     0x00
    #define ADS_TIMEOUT_ENABLE      0x04
    #define ADS_CRC_DISABLE         0x00
    #define ADS_CRC_ENABLE          0x02
    #define ADS_CRC_MASK            0x02
    #define ADS_SENDSTATUS_DISABLE  0x00
    #define ADS_SENDSTATUS_ENABLE   0x01
    #define ADS_SENDSTATUS_MASK     0x01

/* ADS124S08 Register 0xA (OFCAL0) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                                        OFC[7:0]                                             |
 *-----------------------------------------------------------------------------------------------
 */
    // OFCAL0 register address
    #define REG_ADDR_OFCAL0         ((uint16_t) 0x0A)    // adapted from 8 bits to 16

    /** OFCAL0 default (reset) value */
    #define OFCAL0_DEFAULT          ((uint16_t) 0x00)    // adapted from 8 bits to 16



/* ADS124S08 Register 0xB (OFCAL1) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                                        OFC[15:8]                                            |
 *-----------------------------------------------------------------------------------------------
 */
    // OFCAL1 register address
    #define REG_ADDR_OFCAL1         ((uint16_t) 0x0B)   // adapted from 8 bits to 16

    /** OFCAL1 default (reset) value */
    #define OFCAL1_DEFAULT          ((uint16_t) 0x00)   // adapted from 8 bits to 16


/* ADS124S08 Register 0xC (OFCAL2) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                                        OFC[23:16]                                           |
 *-----------------------------------------------------------------------------------------------
 */
    // OFCAL2 register address
    #define REG_ADDR_OFCAL2         ((uint16_t) 0x0C)   // adapted from 8 bits to 16

    /** OFCAL2 default (reset) value */
    #define OFCAL2_DEFAULT          ((uint16_t) 0x00)   // adapted from 8 bits to 16


/* ADS124S08 Register 0xD (FSCAL0) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                                        FSC[7:0]                                             |
 *-----------------------------------------------------------------------------------------------
 */
    // FSCAL0 register address
    #define REG_ADDR_FSCAL0         ((uint16_t) 0x0D)   // adapted from 8 bits to 16

    /** FSCAL0 default (reset) value */
    #define FSCAL0_DEFAULT          ((uint16_t) 0x00)   // adapted from 8 bits to 16


/* ADS124S08 Register 0xE (FSCAL1) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                                        FSC[15:8]                                            |
 *-----------------------------------------------------------------------------------------------
 */
    // FSCAL1 register address
    #define REG_ADDR_FSCAL1         ((uint16_t) 0x0E)   // adapted from 8 bits to 16

    /** FSCAL1 default (reset) value */
    #define FSCAL1_DEFAULT          ((uint16_t) 0x00)   // adapted from 8 bits to 16


/* ADS124S08 Register 0xF (FSCAL2) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                                        FSC[23:16]                                           |
 *-----------------------------------------------------------------------------------------------
 */
    // FSCAL2 register address
    #define REG_ADDR_FSCAL2         ((uint16_t) 0x0F)   // adapted from 8 bits to 16

    /** FSCAL2 default (reset) value */
    #define FSCAL2_DEFAULT          ((uint16_t) 0x40)   // adapted from 8 bits to 16


/* ADS124S08 Register 0x10 (GPIODAT) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|                     DIR[3:0]                 |                  DAT[3:0]                    |
 *-----------------------------------------------------------------------------------------------
 */
    // GPIODAT register address
    #define REG_ADDR_GPIODAT        ((uint16_t) 0x10)   // adapted from 8 bits to 16

    /** GPIODAT default (reset) value */
    #define GPIODAT_DEFAULT         ((uint16_t) 0x00)   // adapted from 8 bits to 16

    // Define GPIO direction (0-Output; 1-Input) here
    #define ADS_GPIO0_DIR_INPUT     0x10
    #define ADS_GPIO1_DIR_INPUT     0x20
    #define ADS_GPIO2_DIR_INPUT     0x40
    #define ADS_GPIO3_DIR_INPUT     0x80


/* ADS124S08 Register 0x11 (GPIOCON) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0  |
 *-----------------------------------------------------------------------------------------------
 *|    0     |     0     |     0     |     0     |                    CON[3:0]                  |
 *-----------------------------------------------------------------------------------------------
 */
    // GPIOCON register address
    #define REG_ADDR_GPIOCON        ((uint16_t) 0x11)   // adapted from 8 bits to 16

    /** GPIOCON default (reset) value */
    #define GPIOCON_DEFAULT         ((uint16_t) 0x00)   // adapted from 8 bits to 16

    // Define GPIO configuration (0-Analog Input; 1-GPIO) here
    #define ADS_GPIO0_DIR_INPUT     0x10
    #define ADS_GPIO1_DIR_INPUT     0x20
    #define ADS_GPIO2_DIR_INPUT     0x40
    #define ADS_GPIO3_DIR_INPUT     0x80

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

inline uint16_t xferWord(uint16_t tx)
{
    uint16_t rx;
    /* Set up data for the next xmit */
    SpiaRegs.SPITXBUF = (tx << 8);
    /* Wait for data to appear */
    while(SpiaRegs.SPIFFRX.bit.RXFFST!=1){ };
    /* Grab that data*/
    rx = SpiaRegs.SPIRXBUF;
    return rx;
}
//    Function prototypes
    uint16_t getRegisterValue( uint16_t address );
    void regWrite(uint16_t regnum, uint16_t data);
    uint16_t regRead(uint16_t regnum);
    void readRTDtemp(void);
    bool adcStartupRoutine(ADCchar_Set *adcChars);
    void restoreRegisterDefaults(void);
    void startConversions(void);
    void stopConversions(void);
    int32_t readConvertedData(uint16_t status[], readMode mode );
    void clearChipSelect(void);
    void setChipSelect(void);

//    ADS124S08 Datasheet code sequence example
//
//    Power-up so that all supplies reach minimum operating levels;
//    Delay for a minimum of 2.2 ms to allow power supplies to settle and power-up reset to complete;
//    DELAY_US(2200);

//    Configure the SPI interface of the microcontroller to SPI mode 1 (CPOL = 0, CPHA =1);

//    If the CS pin is not tied low permanently, configure the microcontroller GPIO connected to CS as an
//    output;

//    Configure the microcontroller GPIO connected to the DRDY pin as a falling edge triggered interrupt
//    input;

//
//    Set CS to the device low;
//    Delay for a minimum of td(CSSC);
//void clearChipSelect(void);

//    Send the RESET command (06h) to make sure the device is properly reset after power-up; //Optional
//void spi_send(Uint16 data);

//    Delay for a minimum of 4096 · tCLK; tCLK = 1/fCLK
//    DELAY_US(DELAY_RESET);

//    Read the status register using the RREG command to check that the RDY bit is 0; //Optional
//    Clear the FL_POR flag by writing 00h to the status register; //Optional
//    Write the respective register configuration with the WREG command;
//
//    Send 42 // WREG starting at 02h address
//    05 // Write to 6 registers
//    12 // Select AINP = AIN1 and AINN = AIN2
//    0A // PGA enabled, Gain = 4
//    14 // Continuous conversion mode, low-latency filter, 20-SPS data rate
//    12 // Positive reference buffer enabled, negative reference buffer disabled
//    // REFP0 and REFN0 reference selected, internal reference always on
//    07 // IDAC magnitude set to 1 mA
//    F0; // IDAC1 set to AIN0, IDAC2 disabled

//    Set CS high;
//void setChipSelect(void);

//    For verification, read back all configuration registers with the RREG command;
//
//    Set CS low; // For verification, read back configuration registers
//void clearChipSelect(void);

//    Send 22 // RREG starting at 02h address
//    05 // Read from 6 registers
//    00 00 00 00 00 00; // Send 6 NOPs for the read
//
//    Set CS high;
//    Set CS low;
//
//    Send the START command (08h) to start converting in continuous conversion mode;
//    Delay for a minimum of td(SCCS);
//    Clear CS to high (resets the serial interface);
//    Loop
//    {
//    Wait for DRDY to transition low;
//    Take CS low;
//    Delay for a minimum of td(CSSC);
//    Send the RDATA command;
//    Send 12 // Send RDATA command
//
//    Send 24 SCLK rising edges to read out conversion data on DOUT/DRDY;
//    00 00 00; // Send 3 NOPs (24 SCLKs) to clock out data
//
//    Delay for a minimum of td(SCCS);
//    Clear CS to high;
//    }
//    Take CS low;
//    Delay for a minimum of td(CSSC);
//    Send the STOP command (0Ah) to stop conversions and put the device in standby mode;
//    Delay for a minimum of td(SCCS);
//    Clear CS to high;

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

/** Register bit checking macros...
 *  Return true if register bit is set (since last read or write).
 */
#define IS_SENDSTAT_SET     ((bool) (getRegisterValue(REG_ADDR_SYS) & ADS_SENDSTATUS_MASK))
#define IS_CRC_SET          ((bool) (getRegisterValue(REG_ADDR_SYS) & ADS_CRC_MASK))
#endif /* ADS124S08_H_ */
