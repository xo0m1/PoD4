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
#define MAX_BUF 1024


/********************* LOCAL Function Prototypes **********************/
int testADC(void);
void exitingFunction(int signo);

/*************************** Globals **********************************/

static pthread_t *p1;
static pthread_t *p2;
static pthread_t *p3;

sem_t mutex_adc;
sem_t mutex_gpio;

int DeltaDistance = 0;

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
	
	
	// Register a function to be called when SIGINT occurs
	//gpioSetSignalFunc(SIGINT, (gpioSignalFunc_t)exitingFunction);
	/*
	int fd;
    char * myfifo = "/tmp/blinkDfifo";
    char buf[MAX_BUF];

    // open, read, and display the message from the FIFO 
    fd = open(myfifo, O_RDONLY);
    
    for(int i = 0; i < 5; i++)
    {
		read(fd, buf, 5); //MAX_BUF);
		fprintf(stderr,"Received: %s\n", buf); 
		//fflush(stdout); 
	}
	
	sleep(4);
    close(fd); 
    
    return 0;
	*/



	//p1 = gpioStartThread(pulseSensor_task, (void *)"thread 1 - PULSE SENSOR"); 
	//sleep(1);
	
	p2 = gpioStartThread(pressureSensor_task, (void *)"thread 2 - PRESSURE SENSOR"); 
	sleep(1);

	p3 = gpioStartThread(proximitySensor_task, (void *)"thread 3 - PROXIMITY SENSOR"); 
	sleep(1);
  
  
	//pthread_join(*p1, NULL);
    pthread_join(*p2, NULL);
    pthread_join(*p3, NULL);
    

	
	// terminate the gpio module
	gpioTerminate();
	
	// destroy the semaphore
	sem_destroy(&mutex_gpio);
	sem_destroy(&mutex_adc);
	
	return 0;
	
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
	int ret = ads1015_init(&adc);
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
