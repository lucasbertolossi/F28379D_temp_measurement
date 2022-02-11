/*
 * Peripheral_Setup.h
 *
 *  Created on: 23 de nov de 2021
 *      Author: lbert
 *
 * GPIO 2 (J4 pin 38): Pwm2A        GPIO3 (J4 pin 37): Pwm2B
 * GPIO 18 (J1 pin 4): RPWM_EN      GPIO19 (J1 pin 3): LPWM_EN; IBT-H BRIDGE
 * GPIO 25 (J6 pin 51): XINT2       GPIO26 (J6 pin 53): XINT3
 */

#ifndef PERIPHERAL_SETUP_H_
#define PERIPHERAL_SETUP_H_

#include "F28x_Project.h"
#include <stdio.h>

// Defines
#define EPWM2_TIMER_TBPRD  19999  // Period register, assymetric 10khz
#define EPWM2_MAX_CMPA     20000
#define EPWM2_MIN_CMPA       0
#define EPWM2_MAX_CMPB     20000
#define EPWM2_MIN_CMPB       0
#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0
#define AUTOCONVERT           1   // 1 = Turn auto-conversion ON
                                  // 0 = Turn auto-conversion OFF


void Setup_ePWM(void);
void Setup_ePWM_Gpio(void);
void Setup_Buttons_Gpio(void);




#endif /* PERIPHERAL_SETUP_H_ */
