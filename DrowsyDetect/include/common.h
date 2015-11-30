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
#include <unistd.h>
#include <signal.h>
#include <pthread.h>    
#include <semaphore.h>  

#include <pigpio.h>
#include "../include/ads1015.h"

/************************ Macros **************************************/

#define PULSE_SENSOR_GPIO_PIN			21
#define PROXIMITY_SENSOR_GPIO_PIN		16
#define PRESSURE_SENSOR_GPIO_PIN		12

#define PULSE_SENSOR_ADC_CHANNEL		0
#define PROXIMITY_SENSOR_ADC_CHANNEL	1
#define PRESSURE_SENSOR_ADC_CHANNEL		2



#endif
