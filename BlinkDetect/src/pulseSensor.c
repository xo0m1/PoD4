
#include <pigpio.h>


/*

void interruptSetup(){     
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 
*/



// THIS IS THE TIMER 2 INTERRUPT SERVICE ROUTINE. 
// Timer 2 makes sure that we take a reading every 2 miliseconds
void pulseSensor_task(void)
{                      
    int N = 0;
	volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
	volatile unsigned long lastBeatTime = 0;           // used to find the inter beat interval
	volatile int rate[10];                    // used to hold last ten IBI values
	volatile int P =512;                      // used to find peak in pulse wave
	volatile int T = 512;                     // used to find trough in pulse wave
	volatile int thresh = 512;                // used to find instant moment of heart beat
	volatile int amp = 100;                   // used to hold amplitude of pulse waveform
	volatile char firstBeat = 1;        // used to seed rate array so we startup with reasonable BPM
	volatile char secondBeat = 1;       // used to seed rate array so we startup with reasonable BPM
	
	volatile int BPM;                   // used to hold the pulse rate
	volatile int Signal;                // holds the incoming raw data
	volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
	volatile char Pulse = 0;     // true when pulse wave is high, false when it's low
	volatile char QS = 0;        // becomes true when Arduoino finds a beat.
	
	sampleCounter = gpioTick() / 1000;
	
	while (1)
	{
		// read the pulse sensor signal
		//int Signal = readFromADC();
		int Signal = 0;	
		
		// keep track of the time in mS with this variable
		sampleCounter += 2;   

		
		N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

	//  find the peak and trough of the pulse wave
	// avoid dichrotic noise by waiting 3/5 of last IBI
		if( (Signal < thresh) && (N > (IBI/5)*3) )
		{  
			// T is the trough
			if (Signal < T)
			{               
				// keep track of lowest point in pulse wave 
				T = Signal;                         
			}
		}
		  
		// thresh condition helps avoid noise
		if(Signal > thresh && Signal > P)
		{          
			P = Signal;                             // P is the peak
		}                                        // keep track of highest point in pulse wave
		
		//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
		// signal surges up in value every time there is a pulse
		if (N > 250)
		{                                   // avoid high frequency noise
			if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
			{        
				Pulse = true;                               // set the Pulse flag when we think there is a pulse
				//digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
				IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
				lastBeatTime = sampleCounter;               // keep track of time for next pulse
			 
				if(firstBeat)
				{                         // if it's the first time we found a beat, if firstBeat == TRUE
					firstBeat = 0;                 // clear firstBeat flag
					return;                            // IBI value is unreliable so discard it
				}   
				if(secondBeat)
				{                        // if this is the second beat, if secondBeat == TRUE
					secondBeat = 0;                 // clear secondBeat flag
					for(int i=0; i<=9; i++)
					{         // seed the running total to get a realisitic BPM at startup
						rate[i] = IBI;                      
					}
				}
			  
				// keep a running total of the last 10 IBI values
				int runningTotal = 0;                   // clear the runningTotal variable    

				for(int i=0; i<=8; i++)
				{                // shift data in the rate array
					rate[i] = rate[i+1];              // and drop the oldest IBI value 
					runningTotal += rate[i];          // add up the 9 oldest IBI values
				}
			
				rate[9] = IBI;                          // add the latest IBI to the rate array
				runningTotal += rate[9];                // add the latest IBI to runningTotal
				//runningTotal /= 10;                     // average the last 10 IBI values
				float rtotal = runningTotal / 10.0;
				 
				BPM = (int)(60000.0/rtotal);               // how many beats can fit into a minute? that's BPM!
				QS = 1;                              // set Quantified Self flag 
				// QS FLAG IS NOT CLEARED INSIDE THIS ISR
			}                       
		}

		// when the values are going down, the beat is over
		if (Signal < thresh && Pulse == true)
		{     
			//digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
			Pulse = 0;                         // reset the Pulse flag so we can do it again
			amp = P - T;                           // get amplitude of the pulse wave
			thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
			P = thresh;                            // reset these for next time
			T = thresh;
		}
	  
		// if 2.5 seconds go by without a beat
		if (N > 2500)
		{                             
			thresh = 512;                          // set thresh default
			P = 512;                               // set P default
			T = 512;                               // set T default
			lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
			firstBeat = 1;                      // set these to avoid noise
			secondBeat = 1;                     // when we get the heartbeat back
		}  
		
		// sleep for 2 ms
		struct timespec ts, rem;  
		ts.tv_sec = 0;
		ts.tv_nsec = 2 * 1E6;
		while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
		{
			/* copy remaining time to ts */
			ts.tv_sec  = rem.tv_sec;
			ts.tv_nsec = rem.tv_nsec;
		}
		
	} // end of while loop
}



