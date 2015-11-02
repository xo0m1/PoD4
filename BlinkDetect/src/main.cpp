// 



#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "../include/blink_detection.h"
#include "../include/ads1015.h"

using namespace std;

// Main function, defines the entry point for the program.
int main( int argc, char** argv )
{
	
	blink_detection();
	
	
	////////////////////////
	
	/*
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
	*/
	
	
	return 0;
	
}
