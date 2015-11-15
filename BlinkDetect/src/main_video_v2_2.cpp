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


#define BUFFER_SIZE		3

cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eye_cascade;


void trackEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);
double detectEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);
double findEyes(cv::Mat frame_gray, cv::Rect face) ;


////////////////////////////
// Debugging
const bool kPlotVectorField = false;

// Size constants
const int kEyePercentTop = 25;
const int kEyePercentSide = 13;
const int kEyePercentHeight = 30;
const int kEyePercentWidth = 35;

// Preprocessing
const bool kSmoothFaceImage = false;
const float kSmoothFaceFactor = 0.005;

// Algorithm Parameters
const int kFastEyeWidth = 50;
const int kWeightBlurSize = 5;
const bool kEnableWeight = true;
const float kWeightDivisor = 1.0;
const double kGradientThreshold = 50.0;

// Postprocessing
const bool kEnablePostProcess = true;
const float kPostProcessThreshold = 0.97;

// Eye Corner
const bool kEnableEyeCorner = false;
////////////////////////////

using namespace std;
using namespace cv;


int main()
{
	// Load the cascade classifiers
	// Make sure you point the XML files to the right path, or 
	// just copy the files from [OPENCV_DIR]/data/haarcascades directory
	face_cascade.load("xml/haarcascade_frontalface_alt2.xml");
	eye_cascade.load("xml/haarcascade_eye.xml");
	
	
	
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
    //Camera.set(CV_CAP_PROP_EXPOSURE, raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW); //-1 is auto, values range from 0 to 100
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
	
	//const int bufferSize = 5; 
	double sumBuffer[BUFFER_SIZE];
	double sum = 0;
	int index = 0;

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
			
			//cout <<" g " << endl;
		// Resizing the image to a smaller size
		resize(image_cam, image, size); 
		

		// Convert to grayscale and 
		// adjust the image contrast using histogram equalization
		cv::Mat gray;
		//cv::cvtColor(image, gray, CV_BGR2GRAY);
		cv::equalizeHist(image, gray);
		
		
				
		// Find the eyes
		sum = detectEye(gray, eye_tpl, eye_bb);	
		
		/*
		double avg = 0;
		for(int j = 0; j < BUFFER_SIZE; j++)
		{
			avg += sumBuffer[j];
		}
		avg = avg / BUFFER_SIZE;
		
		
		
		if (sum >= (avg * 1.13) || sum <= (avg * 0.87))		
		{
			cout << "blink!!" << endl;
		}
		else
		{
			sumBuffer[index++] = sum;
		}
		
		
		if (index == BUFFER_SIZE) { index = 0;}		
			*/
		
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
double detectEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect)
{
	double sum = 0;	
	std::vector<cv::Rect> faces, eyes;
	face_cascade.detectMultiScale(im, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

	for (unsigned int i = 0; i < faces.size(); i++)
	{
		//cv::Mat face = im(faces[i]);			
		sum = findEyes(im, faces[i]);			
	}

	return sum;
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



// from: http://thume.ca/projects/2012/11/04/simple-accurate-eye-center-tracking-in-opencv/,
// https://github.com/trishume/eyeLike/blob/master/src/main.cpp
double findEyes(cv::Mat frame_gray, cv::Rect face) 
{
	cv::Mat faceROI = frame_gray(face);
	cv::Mat debugFace = faceROI;
	
	/*
	if (kSmoothFaceImage) {
	double sigma = kSmoothFaceFactor * face.width;
	GaussianBlur( faceROI, faceROI, cv::Size( 0, 0 ), sigma);
	}
	* */
	
	
	//-- Find eye regions and draw them
	int eye_region_width = face.width * (kEyePercentWidth/100.0);
	int eye_region_height = face.width * (kEyePercentHeight/100.0);
	int eye_region_top = face.height * (kEyePercentTop/100.0);
	cv::Rect leftEyeRegion(face.width*(kEyePercentSide/100.0),
						 eye_region_top,eye_region_width,eye_region_height);
	cv::Rect rightEyeRegion(face.width - eye_region_width - face.width*(kEyePercentSide/100.0),
						  eye_region_top,eye_region_width,eye_region_height);
	
	// Show the rectangle on the left eye
	cv::rectangle(faceROI, leftEyeRegion, CV_RGB(0,255,0));
	cv::imshow("face", faceROI);
	////////////////////////////////	
	
	cv::Mat eyeL = faceROI(leftEyeRegion);
	
	cv::Mat eyeBin, eyeSmooth;
	cv::GaussianBlur(faceROI(leftEyeRegion),eyeSmooth,cv::Size(5,5),1.5);	
	cv::threshold(eyeSmooth, eyeBin, 67, 255, CV_THRESH_BINARY);
	
	
	//cv::Scalar sum = cv::sum(eyeBin);
	//cout << sum[0] << endl;	
	//cv::Scalar tempVal = mean( eyeBin );
	//float mean = tempVal.val[0];
	//cout << mean << endl;
	
	
	std::vector<std::vector<cv::Point> > contours;
	cv::Mat contourOutput = eyeBin.clone();
	cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    //Draw the contours
    cv::Mat contourImage(eyeBin.size(), CV_8UC1, cv::Scalar(0,0,0));
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);
    for (size_t idx = 0; idx < contours.size(); idx++) 
    {
        cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
    }
    
    // Show the contour image of the left eye
	//cv::imshow("Contours", contourImage);	
	
	
	// Get contours info
	//cout << "# of contour points: " << contours[0].size() << endl ;
	//cout << " Area: " << contourArea(contours[0]) << endl;
	
	
	
	static int count = 1;
	static bool prev_blink = false;
	if (contourArea(contours[0]) > 500 && prev_blink == false)
	{
		cout << "blink # " << count++ << endl;
		prev_blink = true;
	}
	else
	{
		prev_blink = false;
	}
	
	
	
	/*
	for(unsigned int i=0;i < contours.size();i++)
	{
		cout << "# of contour points: " << contours[i].size() << endl ;
		
		for(unsigned int j=0;j<contours[i].size();j++)
		{
			cout << "Point(x,y)=" << contours[i][j] << endl;
		}
		cout << " Area: " << contourArea(contours[i]) << endl;
	}
	* */
	
	
	//imshow("face", eyeBin);
	
	return 1; //sum[0];
	
	
	
	
	
	/*
	//-- Find Eye Centers
	cv::Point leftPupil = findEyeCenter(faceROI,leftEyeRegion,"Left Eye");
	cv::Point rightPupil = findEyeCenter(faceROI,rightEyeRegion,"Right Eye");
	
	
	// get corner regions
	cv::Rect leftRightCornerRegion(leftEyeRegion);
	leftRightCornerRegion.width -= leftPupil.x;
	leftRightCornerRegion.x += leftPupil.x;
	leftRightCornerRegion.height /= 2;
	leftRightCornerRegion.y += leftRightCornerRegion.height / 2;
	cv::Rect leftLeftCornerRegion(leftEyeRegion);
	leftLeftCornerRegion.width = leftPupil.x;
	leftLeftCornerRegion.height /= 2;
	leftLeftCornerRegion.y += leftLeftCornerRegion.height / 2;
	cv::Rect rightLeftCornerRegion(rightEyeRegion);
	rightLeftCornerRegion.width = rightPupil.x;
	rightLeftCornerRegion.height /= 2;
	rightLeftCornerRegion.y += rightLeftCornerRegion.height / 2;
	cv::Rect rightRightCornerRegion(rightEyeRegion);
	rightRightCornerRegion.width -= rightPupil.x;
	rightRightCornerRegion.x += rightPupil.x;
	rightRightCornerRegion.height /= 2;
	rightRightCornerRegion.y += rightRightCornerRegion.height / 2;
	rectangle(debugFace,leftRightCornerRegion,200);
	rectangle(debugFace,leftLeftCornerRegion,200);
	rectangle(debugFace,rightLeftCornerRegion,200);
	rectangle(debugFace,rightRightCornerRegion,200);
	// change eye centers to face coordinates
	rightPupil.x += rightEyeRegion.x;
	rightPupil.y += rightEyeRegion.y;
	leftPupil.x += leftEyeRegion.x;
	leftPupil.y += leftEyeRegion.y;
	// draw eye centers
	circle(debugFace, rightPupil, 3, 1234);
	circle(debugFace, leftPupil, 3, 1234);
	
	//-- Find Eye Corners
	if (kEnableEyeCorner) 
	{
		cv::Point2f leftRightCorner = findEyeCorner(faceROI(leftRightCornerRegion), true, false);
		leftRightCorner.x += leftRightCornerRegion.x;
		leftRightCorner.y += leftRightCornerRegion.y;
		cv::Point2f leftLeftCorner = findEyeCorner(faceROI(leftLeftCornerRegion), true, true);
		leftLeftCorner.x += leftLeftCornerRegion.x;
		leftLeftCorner.y += leftLeftCornerRegion.y;
		cv::Point2f rightLeftCorner = findEyeCorner(faceROI(rightLeftCornerRegion), false, true);
		rightLeftCorner.x += rightLeftCornerRegion.x;
		rightLeftCorner.y += rightLeftCornerRegion.y;
		cv::Point2f rightRightCorner = findEyeCorner(faceROI(rightRightCornerRegion), false, false);
		rightRightCorner.x += rightRightCornerRegion.x;
		rightRightCorner.y += rightRightCornerRegion.y;
		circle(faceROI, leftRightCorner, 3, 200);
		circle(faceROI, leftLeftCorner, 3, 200);
		circle(faceROI, rightLeftCorner, 3, 200);
		circle(faceROI, rightRightCorner, 3, 200);
	}
	*/
	
	//imshow("face", faceROI);
	//  cv::Rect roi( cv::Point( 0, 0 ), faceROI.size());
	//  cv::Mat destinationROI = debugImage( roi );
	//  faceROI.copyTo( destinationROI );
}

