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
	face_cascade.load("haarcascade_frontalface_alt2.xml");
	eye_cascade.load("haarcascade_eye.xml");
	
	
	
    // Open webcam
    cv::Mat image_cam;
    raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );   
    
    
    cout<<"Opening Camera..."<<endl;
    int cam_open = Camera.open();
    if (!cam_open) 
    {
		cerr<<"Error opening the camera"<<endl;
		return 1;
	}
	if (face_cascade.empty() || eye_cascade.empty())
	{
		cerr<< "Error in template load." << endl;
		return 1;
	}
    
    // Camera settings
    
    // native resulution is 1280 X 900
    //Camera.set(CV_CAP_PROP_BRIGHTNESS, 50); // values range from 0 to 100
    //Camera.set(CV_CAP_PROP_GAIN, -1); // values range from 0 to 100
    Camera.set(CV_CAP_PROP_EXPOSURE, raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW); //-1 is auto, values range from 0 to 100
    //Camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 50); //values range from 0 to 100, -1 auto whitebalance
    //Camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 50); //values range from 0 to 100,  -1 auto whitebalance


	//cv::Mat frame;
	cv::Mat eye_tpl;
	cv::Rect eye_bb;
	
	//Size size(640,480);
	Size size(320,240);
	cv::Mat image;
	int counter = 0;
	
	cout << "Let's begin!" << endl;

	while (cv::waitKey(15) != 'q')
	{
		//cap >> frame;
		Camera.grab();
		Camera.retrieve (image_cam);
		if (image_cam.empty())
		{
			cout << "no image read from camera -- terminating" << endl;
			break;
		}
			
		// Resizing the image to a smaller size
		resize(image_cam, image, size); 
		

		// Convert to grayscale and 
		// adjust the image contrast using histogram equalization
		cv::Mat gray;
		//cv::cvtColor(image, gray, CV_BGR2GRAY);
		cv::equalizeHist(image, gray);
				
		
		detectEye(gray, eye_tpl, eye_bb);		
		
		if (eye_bb.width != 0 || eye_bb.height != 0)
		{					
			// Tracking stage with template matching
			trackEye(gray, eye_tpl, eye_bb);

			// Draw bounding rectangle for the eye
			cv::rectangle(gray, eye_bb, CV_RGB(0,255,0));
			
			counter++;
		}
		
		if (counter == 5)
		{
			eye_bb.width = 0;
			eye_bb.height = 0;
			counter = 0;
		}
		

		// Display video
		cv::imshow("video", gray);
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

	for (unsigned int i = 0; i < faces.size(); i++)
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


