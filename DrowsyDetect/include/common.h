/*
 =============================================================================
 Author      : William A Irizarry
 Version     : 1
 Description : Common header file for the project
 ============================================================================
 */

 

#ifndef _COMMON_H_
#define _COMMON_H_


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>

#include <pigpio.h>
#include "../include/ads1015.h"

/************************ Macros **************************************/

#define GPIO_PIN						4
#define PROXIMITY_SENSOR_GPIO_PIN		22
#define PRESSURE_SENSOR_GPIO_PIN		6

#define PULSE_SENSOR_ADC_CHANNEL		0
#define PROXIMITY_SENSOR_ADC_CHANNEL	1
#define PRESSURE_SENSOR_ADC_CHANNEL		2



#endif
