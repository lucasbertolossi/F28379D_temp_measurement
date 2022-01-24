/*
 * Peripheral_Setup.h
 *
 *  Created on: 23 de nov de 2021
 *      Author: lbert
 *
 * GPIO 2: Pwm2A        GPIO3: Pwm2B
 */

#ifndef PERIPHERAL_SETUP_H_
#define PERIPHERAL_SETUP_H_

#include "F28x_Project.h"
#include <stdio.h>

// Defines
#define EPWM2_TIMER_TBPRD  50000  // Period register
#define EPWM2_MAX_CMPA     50000
#define EPWM2_MIN_CMPA       0
//#define EPWM2_MAX_CMPB     1950
//#define EPWM2_MIN_CMPB       50
#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0


void Setup_ePWM(void);
void Setup_ePWM_Gpio(void);




#endif /* PERIPHERAL_SETUP_H_ */
