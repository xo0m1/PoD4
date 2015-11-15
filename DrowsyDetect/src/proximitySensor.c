/*
 =============================================================================
 Author      : William A Irizarry
 Version     : 1
 Description : Implementation of the proximity sensor interface module.  
 ============================================================================
 */




#include "../include/common.h"


/************************ Macros **************************************/



/********************* LOCAL Function Prototypes **********************/
static long map(long x, long in_min, long in_max, long out_min, long out_max);


/*********************** Function Definitions *************************/

void *proximitySensor_task(void *arg)
{
	
	printf("proximity: Task Started %s\n", arg);
	
	ads1015_t ads;
	ads1015_init(&ads);
	
	float voltage = 0;
	int scaledVoltage = 0;
	float distance = 0;
	
	while(1)
	{
	
		// read the pulse sensor signal
		voltage = ads1015_getDataFromChannel(&ads, PROXIMITY_SENSOR_ADC_CHANNEL);
		
		// scale the voltage to PWM values
		scaledVoltage = map((int)(voltage * 1000), 400, 4096, 0, 255);
		if (scaledVoltage > 255) scaledVoltage = 255;
		
		// Set the LED brightness value
		gpioPWM(PROXIMITY_SENSOR_GPIO_PIN, scaledVoltage);
		
		// distance calculation in ft (max detected range is 15 meters, and 3.28 ft in 1 m)
		scaledVoltage = map((int)(voltage * 1000), 400, 4096, 0, (15 *  3.28 * 100));
		if (scaledVoltage > 4920) scaledVoltage = 4920;
		
		//printf("d= %d ft\n", scaledVoltage / 100);
		
		
		
		// sleep for 100 ms
		gpioSleep(PI_TIME_RELATIVE, 0, 100000);
	
	}
	
	return 0;
}




/*
** map
**
** Description
**  Map a value from one range of values to another range of values
**
** Input Arguments:
**  in_min		min value of input range
**	in_max		max value of input range
**	out_min		min value of desired output range
**	out_max		min value of desired output range
**	x 			input value
**
** Output Arguments:
**  None
**
** Function Return:
**  value mapped to desired output range
**
** Special Considerations:
**  None
**
**/
static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
