/*
 =============================================================================
 Author      : William A Irizarry
 Version     : 1
 Description : Implementation of the pressure sensor interface module.  
 ============================================================================
 */




#include "../include/common.h"


/************************ Macros **************************************/


/*************************** Globals **********************************/
extern sem_t mutex_adc;
extern sem_t mutex_gpio;

extern int DeltaPressure;
extern int Pressure;


/********************* LOCAL Function Prototypes **********************/
static long map(long x, long in_min, long in_max, long out_min, long out_max);


/*********************** Function Definitions *************************/


void *pressureSensor_task(void *arg)
{
	
	printf("pressure: Task Started %s\n", (char *)arg);
	
	ads1015_t ads;
	ads1015_init(&ads);
	
	float voltage = 0;
	int scaledVoltage = 0;
	//float pressure = 0;
	int prev_value = 0;
	
	int index = 0;
	float avg = 0;
	int delta = 0;
	int window[5];
	for(int i = 0; i < 5; i++)
	{
		// Initialize to middle value
		window[i] = 128;
	}
	
	while(1)
	{
	
		// read the pulse sensor signal
		sem_wait(&mutex_adc);
		voltage = ads1015_getDataFromChannel(&ads, PRESSURE_SENSOR_ADC_CHANNEL);
		usleep(400);
		voltage = ads1015_getDataFromChannel(&ads, PRESSURE_SENSOR_ADC_CHANNEL);
		sem_post(&mutex_adc);
		
		//printf("d= %f\n", voltage);
		
		if (voltage > 0)
		{
		
			// scale the voltage to PWM values
			scaledVoltage = map((int)(voltage * 1000), 1500, 4096, 0, 255);
			if (scaledVoltage > 255) scaledVoltage = 255;
			
			// guard against negative values for PWM
			if (scaledVoltage < 0)
			{
				scaledVoltage = prev_value;
			}
			else
			{
				prev_value = scaledVoltage;
			}
			
			// Set the LED brightness value
			sem_wait(&mutex_gpio);
			gpioPWM(PRESSURE_SENSOR_GPIO_PIN, scaledVoltage);
			sem_post(&mutex_gpio);
			
			
			// determine delta
			avg = 0;
			for(int i = 0; i < 5; i++)
			{
				avg = avg + window[i];
			}
			avg = avg / 5;
			delta = (scaledVoltage - avg);
			//fprintf(stderr,"d= %d\n", delta );
			fprintf(stderr,"d= %d\n", scaledVoltage );
			
			// report to main thread
			DeltaPressure = delta;
			Pressure = scaledVoltage;
			
			
			// Put current value into the averaging window
			window[index++] = scaledVoltage;
			if (index >= 5) index = 0;
			
		}
		
		// sleep for 230 ms
		usleep(230000);
		
	
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
