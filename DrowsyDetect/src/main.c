/*
 =============================================================================
 Author      : William A Irizarry
 Version     : 1
 Description : Main for the drowsiness detection system application. This
				application uses the pigpio library for handling
				GPIO operations and threading, and the raspicam
				library for handling video from the camera.
 ============================================================================
 */

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include "../include/common.h"
#include "../include/pulseSensor.h"
#include "../include/proximitySensor.h"
#include "../include/pressureSensor.h"

/************************ Macros **************************************/
#define MAX_BUF 					5
#define PULSE_CIRCULAR_BUF_SIZE 	15

//#define BLINKDETECT_PIPE



/**************************** Data Types ******************************/


typedef enum {
	PRESSURE_IDLE, 
	PRESSURE_NO_GRIP,
	PRESSURE_ALERT,
	PRESSURE_DISABLE_ALERT
} pressureStatesType;


/********************* LOCAL Function Prototypes **********************/
int testADC(void);
void exitingFunction(int signo);
void *buzzer_task(void *arg);
void sensorFusionAlgorithm(void);
void pressureStateMachine(int counter);

/*************************** Globals **********************************/

sem_t mutex_adc;
sem_t mutex_gpio;
sem_t sem_buzzer;


volatile int DeltaProximity = 0;
volatile int DeltaPressure = 0;
volatile int Pressure = 0;
volatile int Pulse_IBI = 0;
volatile char Pulse_newIBIvalue = 0;


static pthread_t *p1;
static pthread_t *p2;
static pthread_t *p3;
static pthread_t *p4;


static int fifofd = -1;
static char myfifo[] = "/tmp/blinkDfifo";


/************************** Namespaces ********************************/

using namespace std;


/*********************** Function Definitions *************************/





// Main function, defines the entry point for the program.
int main( int argc, char** argv )
{
	// Welcome message
	fprintf(stderr,"drowsyDetect Main\n"); 
	
	
	// initialize the semaphore
	sem_init(&mutex_adc, 0, 1); 
	sem_init(&mutex_gpio, 0, 1);
	sem_init(&sem_buzzer, 0, 0);
	
	// initialize the GPIO module
	if (gpioInitialise() < 0)
	{
		// pigpio initialisation failed.
		fprintf(stderr, "main: Could not initialize GPIOs\n");
		return -1;
	}
	
	// Set GPIOs as output.
	gpioSetMode(PULSE_SENSOR_GPIO_PIN, PI_OUTPUT); 
	gpioSetMode(PRESSURE_SENSOR_GPIO_PIN, PI_OUTPUT);
	gpioSetMode(PROXIMITY_SENSOR_GPIO_PIN, PI_OUTPUT); 
	gpioSetMode(BUZZER_GPIO_PIN, PI_OUTPUT); 
	
	// reset the gpio state of the buzzer
	gpioWrite(BUZZER_GPIO_PIN, 0);		// Set gpio low
	
	
	// Register a function to be called when SIGINT occurs
	//gpioSetSignalFunc(SIGINT, (gpioSignalFunc_t)exitingFunction);
	


	p1 = gpioStartThread(pulseSensor_task, (void *)"thread 1 - PULSE SENSOR"); 
	sleep(1);
	
	p4 = gpioStartThread(buzzer_task, (void *)"thread 4 - BUZZER"); 
	sleep(1);

	p2 = gpioStartThread(pressureSensor_task, (void *)"thread 2 - PRESSURE SENSOR"); 
	sleep(1);

	p3 = gpioStartThread(proximitySensor_task, (void *)"thread 3 - PROXIMITY SENSOR"); 
	sleep(1);

#ifdef BLINKDETECT_PIPE	
	// wait until the named pipe has been created and then open the pipe (FIFO)
	fprintf(stderr,"drowsyDetect Main - Waiting for pipe creation\n"); 
	while(fifofd < 0)
	{
		fifofd = open(myfifo, O_RDONLY);
		usleep(1000);
	}
	
	//This will force the read() to be a non blocking system call
	int flags = fcntl(fifofd, F_GETFL, 0);
	fcntl(fifofd, F_SETFL, flags | O_NONBLOCK);
#endif
	
	// Inform user we are ready
    fprintf(stderr,"drowsyDetect Main - Init OK. Ready to start Sensor Fusion Algorith\n"); 
	
	// run sensor fusion algorithm
	sensorFusionAlgorithm();
	
	
	fprintf(stderr,"drowsyDetect Main - ERROR. Not supposed to be here!\n"); 
	
	
	// Wait for the threads to terminate
	pthread_join(*p1, NULL);
	pthread_join(*p4, NULL);
    pthread_join(*p2, NULL);
    pthread_join(*p3, NULL);
    
	
	// terminate the gpio module
	gpioTerminate();
	
	// destroy the semaphore
	sem_destroy(&mutex_gpio);
	sem_destroy(&mutex_adc);
	sem_destroy(&sem_buzzer);
	
	close(fifofd); 
	
	return 0;
	
}



/*
** sensorFusionAlgorithm
**
** Description
**  Function that executes the sensor fusion algorithm 
**  for the system
**
** Input Arguments:
**  None
**
** Output Arguments:
**  None
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
void sensorFusionAlgorithm(void)
{
	unsigned int counter = 0;
	unsigned int proximityCounter = 0;
	int buzzerFlag = 0;
	char buf[MAX_BUF];
	int pulseIBICircularBuffer[PULSE_CIRCULAR_BUF_SIZE];
	float pulseIBIAvg = 0;
	int index = 0;
	
	// initialize the pulse circular buffer
	for(int i=0; i < PULSE_CIRCULAR_BUF_SIZE; i++)
	{
		pulseIBICircularBuffer[i] = 400;
	}
	
	
	while(1)
	{
		usleep(2000);
		
		
		/////////////////////////
		// Detect approaching object
		/////////////////////////
		if (DeltaProximity > 130 && buzzerFlag == 0)
		{
			sem_post(&sem_buzzer);
			buzzerFlag = 1;
			proximityCounter = counter;
		}
		else
		{
			if ((unsigned int)(counter - proximityCounter) >= 500) // wait for 1 second
			{
				buzzerFlag = 0;
			}
		}
		
		
		/////////////////////////
		// Execute the pressure sensor state machine to detect pressure events
		/////////////////////////
		pressureStateMachine(counter);
	
	
	
		/////////////////////////
		// Check the blink detector for new data
		/////////////////////////
		if (read(fifofd, buf, MAX_BUF) > 0)
		{
			fprintf(stderr,"FIFO Received: %s\n", buf); 
		}
		
		
		
		/////////////////////////
		//  Check the IBI of the pulse sensor
		/////////////////////////
		if (Pulse_newIBIvalue)
		{
			// reset the flag
			Pulse_newIBIvalue = 0;
			pulseIBIAvg = 0.0;
			
			for(int i=0; i < PULSE_CIRCULAR_BUF_SIZE; i++)
			{
				pulseIBIAvg = pulseIBIAvg + (float)pulseIBICircularBuffer[i];
			}
			pulseIBIAvg = pulseIBIAvg / (float)PULSE_CIRCULAR_BUF_SIZE;
			if (Pulse_IBI < (int)(pulseIBIAvg - 100))
			{
				// IBI is less than average IBI
				
			} 
			pulseIBICircularBuffer[index++] = Pulse_IBI;
			if (index >= MAX_BUF)
			{
				index = 0;
			}
			fprintf(stderr,"IBI: %d\n", Pulse_IBI); 
			fprintf(stderr,"IBI avg: %d\n", (int)pulseIBIAvg); 
		}
		
		
		// increase the counter
		counter++;
	}	
}



/*
** buzzer_task
**
** Description
**  Task for triggering the buzzer
**
** Input Arguments:
**  arg		string to be printed at task startup
**
** Output Arguments:
**  None
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
void *buzzer_task(void *arg)
{
	printf("buzzer: Task Started %s\n", (char *)arg);
	
	
	while(1)
	{
		sem_wait(&sem_buzzer);
		gpioWrite(BUZZER_GPIO_PIN, 1);		// Set gpio high
		usleep(1000000);
		gpioWrite(BUZZER_GPIO_PIN, 0);		// Set gpio low
	}
	
	return 0;
}





/*
** pressureStateMachine
**
** Description
**  Function that implements the state machine for detecting
**  pressure sensor events. The state machine will detect a low
**  or no grip on the pressure sensor for at least 3 seconds and
**  alert the user. 
**
** Input Arguments:
**  counter		running counter of 2ms periods elapsed
**
** Output Arguments:
**  None
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
void pressureStateMachine(int counter)
{
	
	static pressureStatesType pressureState = PRESSURE_IDLE;
	static unsigned int pressureCounter = 0;
	//static int buzzerPressureFlag = 0;
	
	switch(pressureState)
	{
		case PRESSURE_IDLE:
		
			if (Pressure < 60 )
			{
				// low or no grip detected -- let's take a closer look
				pressureState = PRESSURE_NO_GRIP;
				pressureCounter = counter;
			}
			else
			{
				// Everything normal
				pressureState = PRESSURE_IDLE;
			}
			break;
			
		case PRESSURE_NO_GRIP:	
		
			if (Pressure < 60 )
			{
				// Still no grip detected
				if ((unsigned int)(counter - pressureCounter) >= 1500)  // wait for 3 seconds
				{
					// low or no grip detected for more than 5 seconds -- alert the user!
					pressureState = PRESSURE_ALERT;	
				}
				else
				{
					pressureState = PRESSURE_NO_GRIP;
				}
			}
			else
			{
				// grip detected - break
				pressureState = PRESSURE_IDLE;
			}
			break;
			
		case PRESSURE_ALERT:
		
			// alert the user -- pressure event
			sem_post(&sem_buzzer);
			//buzzerPressureFlag = 1;
			pressureCounter = counter;
			pressureState = PRESSURE_DISABLE_ALERT;
			break;
			
		case PRESSURE_DISABLE_ALERT:

			if ((unsigned int)(counter - pressureCounter) >= 1000) // wait for 2 seconds
			{
				// done waiting -- we can listen for pressure events again
				//buzzerPressureFlag = 0;
				pressureState = PRESSURE_IDLE;
			}
			else
			{
				// continue ignoring pressure events
				pressureState = PRESSURE_DISABLE_ALERT;
			}
			break;
			
		default:
			fprintf(stderr,"drowsyDetect Main - ERROR - Unknown PRESSURE state!\n"); 
			pressureState = PRESSURE_IDLE;
			break;
	}
	
		//// detect decrease in grip pressure
		//if (Pressure < 60 && buzzerPressureFlag == 0)
		//{
			//sem_post(&sem_buzzer);
			//buzzerPressureFlag = 1;
			//pressureCounter = counter;
			////fprintf(stderr,"!\n");
		//}
		//else
		//{
			////fprintf(stderr,"+\n");
			//if ((unsigned int)(counter - pressureCounter) >= 1000) // wait for 2 second
			//{
				//buzzerPressureFlag = 0;
				////fprintf(stderr,"*\n");
			//}
		//}
}	



/*
** exitingFunction
**
** Description
**  Function that gets called when interrupting or killing 
**  the application. Used for cleaning up.
**
** Input Arguments:
**  signo		signal sent by the user
**
** Output Arguments:
**  None
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
void exitingFunction(int signo)
{
	printf("Exiting...\n"); 
	gpioStopThread(p3); sleep(3);
	gpioStopThread(p2); sleep(3);
	gpioStopThread(p1); sleep(3);
	gpioTerminate();
	printf("Good bye!\n");
	exit(1);
	
}





/*
** testADC
**
** Description
**  Function to test the ADC module
**
** Input Arguments:
**  None
**
** Output Arguments:
**  None
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
int testADC(void)
{
	
	ads1015_t adc;
	int ret = ads1015_init(&adc, GENERIC);
	if (!ret) 
	{
		fprintf(stderr,"ERROR initializing ADC\n");
		return 0;
	}
	
	
	ret = ads1015_changeActiveChannel(&adc, 3);
	if (ret < 0) 
	{
		fprintf(stderr,"ERROR changing channel in ADC\n");
		return 0;
	}
	
	
	usleep(100);
	
	for(int i = 0; i < 1; i++)
	{
		float val = ads1015_getDataFromActiveChannel(&adc);
		if (ret < 0) 
		{
			fprintf(stderr,"ERROR reading from ADC\n");
			return 0;
		}
		printf(" read: %f V\n",val);
	}
	ads1015_close(&adc);
	
	return 1;
	
}
