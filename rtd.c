/**
 * @file rtd.c
 *
 * @brief Resistance Temperature Detector (RTD) Support Routines to Convert converted ADC data to Temperature
 *         Supports: 2-wire, 3-wire, and 4-wire RTDs
 *
 * @copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Software License Agreement
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's Microcontroller and/or Data Converter products.
 * The software is owned by TI and/or its suppliers, and is protected
 * under applicable @copyright Copyright laws.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include "adchal_tidrivers_adapted.h"
#include "inc/rtd.h"

static int Binary_Search(const float *table, float value, int min, int max);
/**
 * @addtogroup group_RTD
 *
 * @{
 */

/************************************************************************************//**
 *
 *
 * @brief Convert_Code2RTD_Resistance()
 *        Convert ADC code to RTD Resistance
 *
 * @param[in] adcChars      ADC characteristic structure that includes: conversion results, PGA gain,
 *                          ADC resolution (8, 12, 14, 16, 20, 24, or 32 bits), ADC voltage reference,
 *                          ADC Offset Calibration Coefficient of ADC (ideal ADC has OffsetCalCoeff = 0),
 *                          ADC Gain Calibration Coefficient (ideal ADC has GainCalCoeff = 1)
 * @param[in] rtdSet        RTD Characteristic set handle
 *
 * @return    RTDres        RTD resistance (ohms) written into rtdSet
 */
float Convert_Code2RTD_Resistance( ADCchar_Set *adcChars, RTD_Set *rtdSet )
{
    float ADCcompValue1, ADCcompValue2;
    float RTDres;

    // Conversion_Result = (Code * Gain_Calibration_Coefficient) + Offset_Calibration_Coefficient
    ADCcompValue1 = ((float) adcChars->adcValue1 * adcChars->gainCalCoeff) + adcChars->offsetCalCoeff;

    switch( rtdSet->wiring ) {

        case Two_Wire_Low_Side_Ref:
        case Two_Wire_High_Side_Ref:
        case Three_Wire_High_Side_Ref_Two_IDAC:
        case Four_Wire_Low_Side_Ref:
        case Four_Wire_High_Side_Ref:
            // RTD_Resistance = (Reference_Resistance * Code) / (Gain * 2^(ADC_Resolution-1))
            RTDres = rtdSet->Rref * ADCcompValue1 / (float) (adcChars->pgaGain * pow(2, adcChars->resolution-1));
            break;

        case Three_Wire_Low_Side_Ref_Two_IDAC:
            // RTD_Resistance = (Reference_Resistance * Code) / (Gain * 2^(ADC_Resolution-2))
            RTDres = rtdSet->Rref * ADCcompValue1 / (float) (adcChars->pgaGain * pow(2, adcChars->resolution-2));;
            break;

        case Three_Wire_High_Side_Ref_One_IDAC:
        case Three_Wire_Low_Side_Ref_One_IDAC:
            // Needs two measurements
            // Conversion_Result = (Code * Gain_Calibration_Coefficient) + Offset_Calibration_Coefficient
            ADCcompValue2 = ((float) adcChars->adcValue2 * adcChars->gainCalCoeff) + adcChars->offsetCalCoeff;
            // RTD_Resistance = (Reference_Resistance * (Code1 - Code2) / (Gain * 2^(ADC_Resolution-1))
            RTDres        = rtdSet->Rref * (ADCcompValue1 - ADCcompValue2) / (adcChars->pgaGain * pow(2, adcChars->resolution-1));
            break;
    }
    return( RTDres );
}


/************************************************************************************//**
 *
 * @brief Binary_Search()
 *        Binary search of a Table
 *
 * @param[in] table Table of data
 * @param[in] value Entry in table of data
 * @param[in] min   Minimum value in Table
 * @param[in] max   Maximum value in Table
 *
 * @return index into Table with match
 *
 */
static int Binary_Search(const float *table, float value, int min, int max)
{
    int midPt = (min + max)/2;

    if ( (midPt == max) || (midPt == min) )
        return( midPt );
    else {
        if ( value < table[midPt])
            return Binary_Search( table, value, min, midPt - 1 );
        else
            return Binary_Search(table, value, midPt + 1, max );

    }
}

/************************************************************************************//**
 *
 * @brief RTD_Linearization()
 *        Converts RTD measured resistance to temperature through interpolation to closest
 *                 temperature value
 *
 * @param[in] rtdSet    RTD Characteristic set handle
 * @param[in] RTDres    RTD resistance
 *
 * @return RTD Temperature (Â°C)
 */
float RTD_Linearization( RTD_Set *rtdSet, float RTDres )
{

#ifndef POLYNOMIAL
    int rtdIndex;
#else
    float tCur, tCur2, tCur3, tNext = 0;
    int   iter;
    float RTDtemp;
#endif


#ifdef POLYNOMIAL  // Polynomial expansion

    // Callendar-Van Dusen equation

    // for T > 0 °C: Rrtd(T) = R0 * [1 + (A * T) + (B * T^2)]
    //   solves to T = -A + Sqrt( A^2 - 4B(1 - Rrtd/R100))/ 2B )
    RTDtemp = (-rtdSet->a + sqrt( rtdSet->a * rtdSet->a - 4 * rtdSet->b * (1 - RTDres/rtdSet->R0)) ) / (2 * rtdSet->b);
    if ( RTDtemp >= 0 )
        return RTDtemp;
    else {
    // for T < 0Â°C: Rrtd(T) = R0 * {1 + (A * T) + (B * T^2) + [C * T^3 * (T â€“ 100)]}
    //   needs interpolation for solution, usually converges within two iterations of:
    //   t(initial) = (Rrtd/R100 - 1)/(A + 100B)
    //   t(n+1) = t(n) - (1 + At(n) + Bt(n)^2 + Ct(n)^3 (t(n) - 100) - Rrtd/R100) / (A +2Bt(n) - 300Ct(n)^2 + 4Ct(n)^3)
        tCur = (RTDres/rtdSet->R0 - 1) / (rtdSet->a + 100 * rtdSet->b);
        for ( iter = 0; iter < 2; iter++) {
            tCur2 = tCur * tCur;
            tCur3 = tCur2 * tCur;
            tNext = tCur - (1 + rtdSet->a * tCur + rtdSet->b * tCur2 + rtdSet->c * tCur3 * (tCur - 100) - RTDres/rtdSet->R0)
                    / (rtdSet->a + 2 * rtdSet->b * tCur - 300 * rtdSet->c * tCur2 + 4 * rtdSet->c * tCur3);
            tCur = tNext;
        }
        return tNext;
    }

#else   // Look-Up Table

    // if previous math gave NAN, return NAN.
    if ( RTDres == NAN ) return NAN;

    if ( RTDres < rtdSet->table[0] )
        return NAN;
    else if ( RTDres <= rtdSet->table[1] )
        return rtdSet->min;
    else if ( RTDres > rtdSet->table[rtdSet->max - rtdSet->min] )
        return NAN;
    if ( RTDres > rtdSet->table[rtdSet->max - rtdSet->min - 1] )
        return rtdSet->max;

    rtdIndex = Binary_Search( rtdSet->table, RTDres, 0, rtdSet->size - 1 );

    // Linear Interpolation
    // tx = m(Resx - Res0) + t0; m = (t1-t0)/(Res1-Res0) = 1/(Res1-Res0)
    // tx = t0 + (Resx - Res0)/(Res1 - Res0)
    if ( (rtdSet->table[rtdIndex+1] - rtdSet->table[rtdIndex]) == 0 ) {      // Both index are the same, choose upper one
        return( rtdIndex + rtdSet->min  );
    } else {
        return( rtdIndex + rtdSet->min + (RTDres - rtdSet->table[rtdIndex])/(rtdSet->table[rtdIndex+1] - rtdSet->table[rtdIndex]) );

    }
#endif
}



float calculate_temperature(float RTDres, char interval){
    float RTDtemp;
    // Testing a few equations
    switch(interval){
        case 'A':       // Considerando as medições de 15 a 40 °C
//            RTDtemp = (RTDres - 100.15)/0.3854;
//            RTDtemp = 0.0021*RTDres*RTDres + 2.1214*RTDres - 233,64;
//            RTDtemp = 2.5945*RTDres - 259.84;
            RTDtemp = 2.5945*RTDres - 259.84 - 0.276 - 0.019;
            break;

        case 'B':       // Considerando as medições de 22 a 28 °C
//            RTDtemp = (RTDres - 100.18)/0.3845;
//            RTDtemp = -0.0108*RTDres*RTDres + 4.9796*RTDres - 391,14
            RTDtemp = 2.6009*RTDres - 260.56;
            break;

        case 'C':       // Considerando as medições de 24 a 26 °C
//            RTDtemp = (RTDres - 100.2)/0.3839;
//            RTDtemp = 0.004*RTDres*RTDres + 1.7258*RTDres - 212.74;
            RTDtemp = 2.6049*RTDres - 261;
            break;

        case 'D':       // Considerando as medições de 20 a 30 °C
//            RTDtemp = (RTDres - 100.15)/0.3855;
//            RTDtemp = 0.0008*RTDres*RTDres + 2.419*RTDres - 250.19;
            RTDtemp = 2.5938*RTDres - 259.78;
            break;

        case 'E':       // Considerando C ( melhor equação para 24 graus C constante) e tirando um offset medio
//            RTDtemp = ((RTDres - 100.2)/0.3839)  - 0.12;
            RTDtemp = ((RTDres - 100.2)/0.3839)  - 0.12 - 0.136;
            break;

        case 'F':       // Considerando as medições de 15 a 40 °C (polinomial ordem 2)
//            RTDtemp = (RTDres - 100.15)/0.3854;
            RTDtemp = 0.0021*RTDres*RTDres + 2.1214*RTDres - 233.64;
            break;

        case 'G':       // Considerando as medições de 22 a 28 °C (polinomial ordem 2)
//            RTDtemp = (RTDres - 100.18)/0.3845;
            RTDtemp = -0.0108*RTDres*RTDres + 4.9796*RTDres - 391.14;
            break;

        case 'H':       // Considerando as medições de 24 a 26 °C (polinomial ordem 2)
//            RTDtemp = (RTDres - 100.2)/0.3839;
            RTDtemp = 0.004*RTDres*RTDres + 1.7258*RTDres - 212.74;
            break;

        case 'I':       // Considerando as medições de 20 a 30 °C (polinomial ordem 2)
//            RTDtemp = (RTDres - 100.15)/0.3855;
            RTDtemp = 0.0008*RTDres*RTDres + 2.419*RTDres - 250.19;
            break;

        case 'J':
//            RTDtemp = 2.6562*RTDres - 266.74;
            RTDtemp = 2.6562*RTDres - 266.74 - 0.159 -0.018;
            break;

        case 'K':
            RTDtemp = -0.0111*RTDres*RTDres + 5.1437*RTDres - 406.53;
            break;
            // J and E
    }

    return RTDtemp;

    }


/** @} // group group_RTD
 *
 */


