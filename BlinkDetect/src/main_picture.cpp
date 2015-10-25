/*
 * main.cpp
 * 
 */
 
#include <ctime>
#include <iostream>
#include <stdio.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <raspicam/raspicamtypes.h>
#include <raspicam/raspicam_cv.h>


using namespace std;
using namespace cv;


int main()
{
	
	cout<< "here" << endl;
	
	time_t timer_begin,timer_end;
	
    // Open webcam
    cv::Mat image_cam;
    int nCount=40; //7  // 40 frames per second
    raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );   
    
    
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) 
    {
		cerr<<"Error opening the camera"<<endl;
		return -1;
	}
    
    // Camera settings
    
    //1280 X 900
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1280); // changing the width does not work
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 900);
    Camera.set(CV_CAP_PROP_BRIGHTNESS, 50); // values range from 0 to 100
    Camera.set(CV_CAP_PROP_GAIN, -1); // values range from 0 to 100
    Camera.set(CV_CAP_PROP_EXPOSURE, raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW); //-1 is auto, values range from 0 to 100
    Camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 50); //values range from 0 to 100, -1 auto whitebalance
    Camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 50); //values range from 0 to 100,  -1 auto whitebalance

    
    cout<<"Capturing "<<nCount<<" frames ...."<<endl;
    time ( &timer_begin );
     for ( int i=0; i<nCount; i++ ) 
     {
		Camera.grab();
		Camera.retrieve ( image);
	}
	time ( &timer_end ); // get current time; same as: timer = time(NULL)  
    double secondsElapsed = difftime ( timer_end,timer_begin );
    cout<< secondsElapsed<<" seconds for "<< nCount<<"  frames : FPS = "<<  ( float ) ( ( float ) ( nCount ) /secondsElapsed ) <<endl;
    //save image 
    cv::imwrite("raspicam_cv_image.jpg",image);
    cout<<"Image saved at raspicam_cv_image.jpg"<<endl;
		
	return 0;
	
	
}

