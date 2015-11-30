/*
 =============================================================================
 Author      : William A Irizarry
 Version     : 1
 Description : Main for the blinkDetect application. This
				application uses  the raspicam
				library for handling video from the camera, and
				OpenCV for the processing of the images.
 ============================================================================
 */



#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "../include/blinkDetectModule.h"


/************************ Macros **************************************/



/********************* LOCAL Function Prototypes **********************/

void exitingFunction(int signo);

/*************************** Globals **********************************/



/************************** Namespaces ********************************/

using namespace std;


/*********************** Function Definitions *************************/





// Main function, defines the entry point for the program.
int main( int argc, char** argv )
{
	
	
	// Register a function to be called when SIGINT occurs
	//
	
	
	blinkDetect_task();
	
	
	

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
	
	cout<< "Good bye!" << endl;
	exit(1);
	
}



