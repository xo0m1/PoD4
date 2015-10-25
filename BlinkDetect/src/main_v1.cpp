/*
 * main.cpp
 * 
 * from: https://github.com/bsdnoobz/opencv-code/blob/master/eye-tracking.cpp
 * 
 * 
 */

/**
 * eye-tracking.cpp:
 * Eye detection and tracking with OpenCV
 *
 * This program tries to detect and tracking the user's eye with webcam.
 * At startup, the program performs face detection followed by eye detection 
 * using OpenCV's built-in Haar cascade classifier. If the user's eye detected
 * successfully, an eye template is extracted. This template will be used in 
 * the subsequent template matching for tracking the eye.
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


cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eye_cascade;


void trackEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);
int detectEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);



using namespace std;
using namespace cv;


int main()
{
	// Load the cascade classifiers
	// Make sure you point the XML files to the right path, or 
	// just copy the files from [OPENCV_DIR]/data/haarcascades directory
	//face_cascade.load("haarcascade_frontalface_alt2.xml");
	face_cascade.load("haarcascade_frontalface_alt.xml");
	eye_cascade.load("haarcascade_eye.xml");
	
	cout<< "here" << endl;
	
	time_t timer_begin,timer_end;
	
    // Open webcam
    cv::Mat image;
    int nCount=7;
    raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    
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
	time ( &timer_end ); /* get current time; same as: timer = time(NULL)  */
    double secondsElapsed = difftime ( timer_end,timer_begin );
    cout<< secondsElapsed<<" seconds for "<< nCount<<"  frames : FPS = "<<  ( float ) ( ( float ) ( nCount ) /secondsElapsed ) <<endl;
    //save image 
    cv::imwrite("raspicam_cv_image.jpg",image);
    cout<<"Image saved at raspicam_cv_image.jpg"<<endl;
	

	
	
	return 0;
	
	cv::VideoCapture cap(0);

	// Check if everything is ok
	if (face_cascade.empty() || eye_cascade.empty() || !cap.isOpened())
	{
		cout<< "Error in image or camera is not available" << endl;
		return 1;
	}

	// Set video to 320x240
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

	cv::Mat frame, eye_tpl;
	cv::Rect eye_bb;

	while (cv::waitKey(15) != 'q')
	{
		cap >> frame;
		if (frame.empty())
			break;

		// Flip the frame horizontally, Windows users might need this
		//cv::flip(frame, frame, 1);

		// Convert to grayscale and 
		// adjust the image contrast using histogram equalization
		cv::Mat gray;
		cv::cvtColor(frame, gray, CV_BGR2GRAY);

		if (eye_bb.width == 0 && eye_bb.height == 0)
		{
			// Detection stage
			// Try to detect the face and the eye of the user
			detectEye(gray, eye_tpl, eye_bb);
		}
		else
		{
			// Tracking stage with template matching
			trackEye(gray, eye_tpl, eye_bb);

			// Draw bounding rectangle for the eye
			cv::rectangle(frame, eye_bb, CV_RGB(0,255,0));
		}

		// Display video
		cv::imshow("video", frame);
	}

	return 0;
}


/**
 * Function to detect human face and the eyes from an image.
 *
 * @param  im    The source image
 * @param  tpl   Will be filled with the eye template, if detection success.
 * @param  rect  Will be filled with the bounding box of the eye
 * @return zero=failed, nonzero=success
 */
int detectEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect)
{
	std::vector<cv::Rect> faces, eyes;
	face_cascade.detectMultiScale(im, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

	for (int i = 0; i < faces.size(); i++)
	{
		cv::Mat face = im(faces[i]);
		eye_cascade.detectMultiScale(face, eyes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(20,20));

		if (eyes.size())
		{
			rect = eyes[0] + cv::Point(faces[i].x, faces[i].y);
			tpl  = im(rect);
		}
	}

	return eyes.size();
}

/**
 * Perform template matching to search the user's eye in the given image.
 *
 * @param   im    The source image
 * @param   tpl   The eye template
 * @param   rect  The eye bounding box, will be updated with the new location of the eye
 */
void trackEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect)
{
	cv::Size size(rect.width * 2, rect.height * 2);
	cv::Rect window(rect + size - cv::Point(size.width/2, size.height/2));
	
	window &= cv::Rect(0, 0, im.cols, im.rows);

	cv::Mat dst(window.width - tpl.rows + 1, window.height - tpl.cols + 1, CV_32FC1);
	cv::matchTemplate(im(window), tpl, dst, CV_TM_SQDIFF_NORMED);

	double minval, maxval;
	cv::Point minloc, maxloc;
	cv::minMaxLoc(dst, &minval, &maxval, &minloc, &maxloc);

	if (minval <= 0.2)
	{
		rect.x = window.x + minloc.x;
		rect.y = window.y + minloc.y;
	}
	else
		rect.x = rect.y = rect.width = rect.height = 0;
}


