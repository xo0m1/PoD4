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



#include "../include/common.h"
#include "../include/blinkDetectModule.h"
#include "../include/pulseSensor.h"
#include "../include/proximitySensor.h"
#include "../include/pressureSensor.h"

/************************ Macros **************************************/



/********************* LOCAL Function Prototypes **********************/
int testADC(void);
void exitingFunction(int signo);

/*************************** Globals **********************************/

static pthread_t *p1;
static pthread_t *p2;
static pthread_t *p3;

/************************** Namespaces ********************************/

using namespace std;


/*********************** Function Definitions *************************/





// Main function, defines the entry point for the program.
int main( int argc, char** argv )
{
	
	
	// initialize the GPIO module
	if (gpioInitialise() < 0)
	{
		// pigpio initialisation failed.
		printf("main: Could not initialize GPIOs\n");
		return -1;
	}
	
	// Set GPIOs as output.
	gpioSetMode(GPIO_PIN, PI_OUTPUT); 
	gpioSetMode(PROXIMITY_SENSOR_GPIO_PIN, PI_OUTPUT); 
	
	
	// Register a function to be called when SIGINT occurs
	//gpioSetSignalFunc(SIGINT, (gpioSignalFunc_t)exitingFunction);
	


	//p1 = gpioStartThread(pulseSensor_task, (void *)"thread 1"); 
	//sleep(3);
	
	//p2 = gpioStartThread(blinkDetect_task, (void *)"thread 2"); 
	//sleep(3);
	
	p3 = gpioStartThread(proximitySensor_task, (void *)"thread 3"); 
	sleep(3);
   
	/*
	p2 = gpioStartThread(myfunc, "thread 2"); sleep(3);
	p3 = gpioStartThread(myfunc, "thread 3"); sleep(3);
	*/


	
	//pulseSensor_task();
	
	//blinkDetect_task();


	while (1)
	{
		gpioSleep(PI_TIME_RELATIVE, 10, 000000);
	}
	
	
	gpioTerminate();
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
	cout<< "Exiting..." << endl;
	//gpioStopThread(p3); sleep(3);
	gpioStopThread(p2); sleep(3);
	gpioStopThread(p1); sleep(3);
	gpioTerminate();
	cout<< "Good bye!" << endl;
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
		cout << "ERROR initializing ADC" << endl;
		return 0;
	}
	
	
	ret = ads1015_changeActiveChannel(&adc, 3);
	if (ret < 0) 
	{
		cout << "ERROR changing channel in ADC" << endl;
		return 0;
	}
	
	
	usleep(100);
	
	for(int i = 0; i < 1; i++)
	{
		float val = ads1015_getDataFromActiveChannel(&adc);
		if (ret < 0) 
		{
			cout << "ERROR reading from ADC" << endl;
			return 0;
		}
		cout << "read: " << val << " V" << endl;
	}
	ads1015_close(&adc);
	
	return 1;
	
}
