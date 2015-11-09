/*
******************************************************************************
 Author      : William A Irizarry-Cruz
 Version     : 1
 Description : Interface module for the ADS1015 chip
 ============================================================================
 */


#include "../include/ads1015.h"




/***************************** Macros *********************************/

#define I2C_BUS							"/dev/i2c-1"

//#define CONFIGURE_ADC						1
#define ADS1015_I2C_ADDRESS					0x48


// Pointer Register
#define  ADS1015_REG_POINTER_MASK        	0x03
#define  ADS1015_REG_POINTER_CONVERT     	0x00
#define  ADS1015_REG_POINTER_CONFIG      	0x01
#define  ADS1015_REG_POINTER_LOWTHRESH   	0x02
#define  ADS1015_REG_POINTER_HITHRESH    	0x03

// Config Register
#define  ADS1015_REG_CONFIG_OS_MASK      	0x8000
#define  ADS1015_REG_CONFIG_OS_SINGLE    	0x8000  // Write: Set to start a single-conversion
#define  ADS1015_REG_CONFIG_OS_BUSY      	0x0000  // Read: Bit = 0 when conversion is in progress
#define  ADS1015_REG_CONFIG_OS_NOTBUSY   	0x8000  // Read: Bit = 1 when device is not performing a conversion

#define  ADS1015_REG_CONFIG_MUX_MASK     	0x7000
#define  ADS1015_REG_CONFIG_MUX_DIFF_0_1 	0x0000  //Differential P = AIN0, N = AIN1 (default)
#define  ADS1015_REG_CONFIG_MUX_DIFF_0_3 	0x1000  //Differential P = AIN0, N = AIN3
#define  ADS1015_REG_CONFIG_MUX_DIFF_1_3 	0x2000  //Differential P = AIN1, N = AIN3
#define  ADS1015_REG_CONFIG_MUX_DIFF_2_3 	0x3000  //Differential P = AIN2, N = AIN3
#define  ADS1015_REG_CONFIG_MUX_SINGLE_0 	0x4000  //Single-ended AIN0
#define  ADS1015_REG_CONFIG_MUX_SINGLE_1 	0x5000  //Single-ended AIN1
#define  ADS1015_REG_CONFIG_MUX_SINGLE_2 	0x6000  //Single-ended AIN2
#define  ADS1015_REG_CONFIG_MUX_SINGLE_3 	0x7000  //Single-ended AIN3

#define  ADS1015_REG_CONFIG_PGA_MASK     	0x0E00
#define  ADS1015_REG_CONFIG_PGA_6_144V   	0x0000  //+/-6.144V range
#define  ADS1015_REG_CONFIG_PGA_4_096V   	0x0200  //+/-4.096V range
#define  ADS1015_REG_CONFIG_PGA_2_048V   	0x0400  //+/-2.048V range (default)
#define  ADS1015_REG_CONFIG_PGA_1_024V   	0x0600  //+/-1.024V range
#define  ADS1015_REG_CONFIG_PGA_0_512V   	0x0800  //+/-0.512V range
#define  ADS1015_REG_CONFIG_PGA_0_256V   	0x0A00  //+/-0.256V range

#define  ADS1015_REG_CONFIG_MODE_MASK    	0x0100
#define  ADS1015_REG_CONFIG_MODE_CONTIN  	0x0000  //Continuous conversion mode
#define  ADS1015_REG_CONFIG_MODE_SINGLE  	0x0100  //Power-down single-shot mode (default)

#define ADS1015_REG_CONFIG_DR_MASK      	0x00E0  
#define ADS1015_REG_CONFIG_DR_128SPS    	0x0000  //128 samples per second
#define  ADS1015_REG_CONFIG_DR_250SPS    	0x0020  //250 samples per second
#define  ADS1015_REG_CONFIG_DR_490SPS    	0x0040  //490 samples per second
#define  ADS1015_REG_CONFIG_DR_920SPS    	0x0060  //920 samples per second
#define  ADS1015_REG_CONFIG_DR_1600SPS   	0x0080  //1600 samples per second (default)
#define  ADS1015_REG_CONFIG_DR_2400SPS   	0x00A0  //2400 samples per second
#define  ADS1015_REG_CONFIG_DR_3300SPS   	0x00C0  //3300 samples per second (also 0x00E0)

#define  ADS1015_REG_CONFIG_CMODE_MASK   	0x0010
#define  ADS1015_REG_CONFIG_CMODE_TRAD   	0x0000  //Traditional comparator with hysteresis (default)
#define  ADS1015_REG_CONFIG_CMODE_WINDOW 	0x0010  //Window comparator

#define  ADS1015_REG_CONFIG_CPOL_MASK    	0x0008
#define  ADS1015_REG_CONFIG_CPOL_ACTVLOW 	0x0000  //ALERT/RDY pin is low when active (default)
#define  ADS1015_REG_CONFIG_CPOL_ACTVHI  	0x0008  //ALERT/RDY pin is high when active

#define  ADS1015_REG_CONFIG_CLAT_MASK    	0x0004  //Determines if ALERT/RDY pin latches once asserted
#define  ADS1015_REG_CONFIG_CLAT_NONLAT  	0x0000  //Non-latching comparator (default)
#define  ADS1015_REG_CONFIG_CLAT_LATCH   	0x0004  //Latching comparator

#define  ADS1015_REG_CONFIG_CQUE_MASK    	0x0003
#define  ADS1015_REG_CONFIG_CQUE_1CONV   	0x0000  //Assert ALERT/RDY after one conversions
#define  ADS1015_REG_CONFIG_CQUE_2CONV   	0x0001  //Assert ALERT/RDY after two conversions
#define  ADS1015_REG_CONFIG_CQUE_4CONV   	0x0002  //Assert ALERT/RDY after four conversions
#define  ADS1015_REG_CONFIG_CQUE_NONE    	0x0003  //Disable the comparator and put ALERT/RDY in high state (default)


#define ENABLE_I2C_PEC						1
#define DISABLE_I2C_PEC						0


/**************************** Data Types ******************************/



/******************** Global Variables ****************************/




/******************** Local Function Prototypes *******************/
static int ads1015_i2cInit(ads1015_t *);
static void byteSwap(uint16_t *word);



/*********************** Function Definitions *************************/



/*
** ads1015_i2cInit
**
** Description
**  Initializes the i2c port for communication with the ads1015 chips.
**  Opens the corresponding i2c device for read and write access.
**
** Input Arguments:
**  A pointer to ads1015_t object.
**
** Output Arguments:
**  The opened file descriptor.
**
** Function Return:
**  1 if operation was successful, 0 if it failed.
**
** Special Considerations:
**  None
**
**/
int ads1015_i2cInit(ads1015_t *chip)
{	
	int fd;	
	
	fd = open(I2C_BUS, O_RDWR );
	if (fd == -1)
	{				
		return 0;		
	}
	
	chip->fd = fd;	
	return 1;	
}



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
**  pointer to ads1015_t object.
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
int ads1015_init(ads1015_t *chip)
{	
	/* Initialize the fields in disp */	
	chip->fd = -1;

		
	/* if we got a valid file descriptor configure the i2c bus */
	if (ads1015_i2cInit(chip))
	{	
		int address = ADS1015_I2C_ADDRESS;
		
		/* Let's register our i2c slave address in the system */
		if (ioctl (chip->fd, I2C_SLAVE, address) < 0 )
		{
			return 0;
		}
		
		/* Disable the error correction */
		if (ioctl(chip->fd, I2C_PEC, DISABLE_I2C_PEC) < 0 )
		{
			return 0;
		}	
		
		chip->active_channel = 0;	
		
#ifdef CONFIGURE_ADC		
		// Read current register state
		//int ret = i2c_smbus_read_word_data(chip->fd, (uint8_t)ADS1015_REG_POINTER_CONFIG);
		//printf("config register init value: 0x%X\n", ret);	
		
		/*
		// Command code for single-shot ADC conversion
		// Disable comparator, Non-latching, Alert/Rdy active low, traditional comparator, single-shot mode
		uint16_t command = ADS1015_REG_CONFIG_CQUE_NONE | ADS1015_REG_CONFIG_CLAT_NONLAT | ADS1015_REG_CONFIG_CPOL_ACTVLOW | ADS1015_REG_CONFIG_CMODE_TRAD | ADS1015_REG_CONFIG_MODE_SINGLE;
		
		// Set sample per seconds, PGA and MUX channel
		command |= ADS1015_REG_CONFIG_DR_250SPS | ADS1015_REG_CONFIG_PGA_4_096V | ADS1015_REG_CONFIG_MUX_SINGLE_0;
		
		//Set 'start single-conversion' bit
		command |= ADS1015_REG_CONFIG_OS_SINGLE;
		*/
		//////////////////
		
		// Command code for Continuous ADC conversion
		// Disable comparator, Non-latching, Alert/Rdy active low, traditional comparator, single-shot mode
		uint16_t command = ADS1015_REG_CONFIG_CQUE_NONE | ADS1015_REG_CONFIG_CLAT_NONLAT | ADS1015_REG_CONFIG_CPOL_ACTVLOW | ADS1015_REG_CONFIG_CMODE_TRAD | ADS1015_REG_CONFIG_MODE_CONTIN;
		
		// Set sample per seconds, PGA and MUX channel
		command |= ADS1015_REG_CONFIG_DR_250SPS | ADS1015_REG_CONFIG_PGA_4_096V | ADS1015_REG_CONFIG_MUX_SINGLE_0;
				
		
		printf("command value: 0x%X\n", command);	
		
		// swap bytes
		byteSwap(&command);
		
		//command = 0xC323;
		//command = 0x23C3; // swapped
		printf("swapped command value: 0x%X\n", command);	
		
		// Write the configuration to the ADC chip (send 0x23C3 for one shot)
		if ( i2c_smbus_write_word_data(chip->fd, ADS1015_REG_POINTER_CONFIG, command) < 0)
		{
			// error
			chip->active_channel = -1;
			return 0;
		}	
		
		usleep(100000);
		
		// read back the config register
		int ret = i2c_smbus_read_word_data(chip->fd, (uint8_t)ADS1015_REG_POINTER_CONFIG);		
		printf("config register new value: 0x%X\n", ret);		
		int byte1 = ret & 0x0000FF00;
		int byte2 = ret & 0x000000FF;	
		byte1 = byte1 >> 8;
		byte2 = byte2 << 8;	
		ret = byte1 | byte2;		
		printf("config register new value, swapped: 0x%X\n", ret);	
#endif		

		return 1;
	}
		
	return 0;
}




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
int ads1015_changeActiveChannel(ads1015_t *chip, int channel)
{

	uint16_t command = 0;
	switch (channel)
	{
		case 0:
			command = ADS1015_REG_CONFIG_MUX_SINGLE_0;
			break;
		case 1:
			command = ADS1015_REG_CONFIG_MUX_SINGLE_1;
			break;
		case 2:
			command = ADS1015_REG_CONFIG_MUX_SINGLE_2;
			break;
		case 3:
			command = ADS1015_REG_CONFIG_MUX_SINGLE_3;
			break;
		default:
			return -1;
			break;
	}

	
	
	// Read current register state
	int ret = i2c_smbus_read_word_data(chip->fd, (uint8_t)ADS1015_REG_POINTER_CONFIG);
	if (ret < 0)
	{
		return -999;
	}	
	
	// Swap the bytes
	byteSwap((uint16_t *)&ret);
	
	//printf("old register value: 0x%X\n", ret);
	
	// Save the current state and modify only the MUX bits
	ret = ret & (~ADS1015_REG_CONFIG_MUX_MASK);
	command = command | ret;
	
	// Swap the bytes again
	byteSwap((uint16_t *)&command);
	
	// Write the new register contents to the register
	if ( i2c_smbus_write_word_data(chip->fd, (uint8_t)ADS1015_REG_POINTER_CONFIG, command) < 0)
	{
		chip->active_channel = -1;
		return -100;
	}
	else
	{
		// success in changing channels
		chip->active_channel = channel;
	}
	
	
	//ret = i2c_smbus_read_word_data(chip->fd, (uint8_t)ADS1015_REG_POINTER_CONFIG);
	//if (ret < 0)
	//{
		//return -999;
	//}	
	//printf("new register value: 0x%X\n", ret);

	return chip->active_channel;
}



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
float ads1015_getDataFromChannel(ads1015_t *chip, int channel)
{	
	
	if (ads1015_changeActiveChannel(chip, channel) < 0)
	{
		return -100;
	}
	
	
	// Read the data from the conversion register	
	int ret = i2c_smbus_read_word_data (chip->fd, (uint8_t)ADS1015_REG_POINTER_CONVERT);
	if (ret < 0 )
	{
		return -100;
	}
	
	// swap the bytes
	byteSwap((uint16_t *)&ret);	
	
	// shift the data right 4 bits, per specs
	ret = ret >> 4;
	
	// multiply by the selected gain in the PGA
	float result = ret * 4096.0;  // programmable gain in mV
	
	// scale it according to our ADC range
	result = result / 2048.0;     // 2^11	
	
	// return the value and scale it back to Volts
	return (result / 1000.0);
}






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
float ads1015_getDataFromActiveChannel(ads1015_t *chip)
{

	// Read the data from the conversion register	
	int ret = i2c_smbus_read_word_data (chip->fd, (uint8_t)ADS1015_REG_POINTER_CONVERT);
	if (ret < 0 )
	{
		return -100;
	}	
	
	// swap the bytes
	byteSwap((uint16_t *)&ret);
	
	
	// shift the data right 4 bits, per specs
	ret = ret >> 4;
	
	// multiply by the selected gain in the PGA
	float result = ret * 4096.0;  // programmable gain in mV
	
	// scale it according to our ADC range
	result = result / 2048.0;     // 2^11	
	
	// return the value and scale it back to Volts
	return (result / 1000.0);
}



/*
** byteSwap
**
** Description
**  Swaps the bytes in a 16 bit variable
**
** Input Arguments:
**   A pointer to uint16_t variable that holds the 
**	 content to be swapped.
**
** Output Arguments:
**  The parameter is modified inside the function
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
void byteSwap(uint16_t *word)
{
	int byte1 = *word & 0x0000FF00;
	int byte2 = *word & 0x000000FF;	
	byte1 = byte1 >> 8;
	byte2 = byte2 << 8;	
	*word = (uint16_t)(byte1 | byte2);	
}



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
void ads1015_close(ads1015_t *chip)
{	
	/* Make sure we have a valid file descriptor */
	if ( chip->fd != -1)
	{
		close(chip->fd);
	}
	return;		
}




