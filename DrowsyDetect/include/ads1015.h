/*
******************************************************************************
 Author      : William A Irizarry
 Version     : 1
 Description : Interface module for the ADS1015 chip
 ============================================================================
 */

#ifndef _ADS1015_H_
#define _ADS1015_H_


#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdlib.h>
#include "i2c-dev.h"





/**************************** Data Types ******************************/



typedef struct {

	int fd;	
	char active_channel;

} ads1015_t;


typedef enum {
	GENERIC, 
	PULSE
} adcType;

/************************ Function Prototypes *************************/



/*
** ads1015_init
** 
** Description
**  Initializes the a ads1015_t chip and
**  opens the corresponding ports to communicate with the devices. This
**	function MUST be called before calling any other ads1015 functions.
**	Failure to do this will have unexpected results.
**
** Input Arguments:
**  Pointer to ads1015_t object.
**	Type of ADC to initialize.
**
** Output Arguments:
**  None
**
** Function Return:
**  1 if operation was successful, 0 if it failed.
**
** Special Considerations:
**  None
**
**/
int ads1015_init(ads1015_t *chip, adcType type);




/*
** ads1015_changeActiveChannel
**
** Description
**  Change the current active channel.
**
** Input Arguments:
**   A pointer to ads1015_t object
**
** Output Arguments:
**  None
**
** Function Return:
**  The active channel
**
** Special Considerations:
**  None
**
**/
int ads1015_changeActiveChannel(ads1015_t *chip, int channel);



/*
** ads1015_getDataFromChannel
**
** Description
**  Reads data from the conversion register. The data comes from the channel
**  selected as part of the function parameters.
**
** Input Arguments:
**   A pointer to ads1015_t object
**
** Output Arguments:
**  None
**
** Function Return:
**  16-bit signed sampled data from the ADC 
**
** Special Considerations:
**  None
**
**/
float ads1015_getDataFromChannel(ads1015_t *chip, int channel);






/*
** ads1015_getDataFromActiveChannel
**
** Description
**  Reads data from the conversion register. The data comes from the last channel
**  selected in the multiplexer.
**
** Input Arguments:
**   A pointer to ads1015_t object
**
** Output Arguments:
**  None
**
** Function Return:
**  16-bit signed sampled data from the ADC 
**
** Special Considerations:
**  None
**
**/
float ads1015_getDataFromActiveChannel(ads1015_t *chip);




/*
** ads1015_close
** 
** Description
**  Close the ads1015 device. This function MUST be called whenever
** 	use of the ads1015 is discontinued.
**
** Input Arguments:
**  A pointer to an ads1015_t object.
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
void ads1015_close(ads1015_t *chip);




#endif /* #ifndef _ADS1015_H_*/





