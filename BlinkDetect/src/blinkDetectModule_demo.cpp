/*
 =============================================================================
 Author      : William A Irizarry
 Version     : 1
 Description : Implementation of the blink detection algorithm. Algorithm
				based on these algorithms: 
				* http://opencv-code.com/tutorials/eye-detection-and-tracking/
				* http://thume.ca/projects/2012/11/04/simple-accurate-eye-center-tracking-in-opencv/
				* http://www.technolabsz.com/2013/05/eye-blink-detection-using-opencv-in.html
 ============================================================================
 */





#include <iostream>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <raspicam/raspicamtypes.h>
#include <raspicam/raspicam_cv.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>



/************************ Macros **************************************/

#define BUFFER_SIZE		3


/********************* LOCAL Function Prototypes **********************/

void trackEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);

//double detectEye(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);
double detectEye(cv::Mat& imColor, cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);


double findEyes_contours(cv::Mat frame_gray, cv::Rect face);

//bool findEyes_classifier(cv::Mat frame_gray, cv::Rect face);
bool findEyes_classifier(cv::Mat frame_color, cv::Mat frame_gray, cv::Rect face);

bool findEyes_hybrid(cv::Mat frame_gray, cv::Rect face);

/*************************** Globals **********************************/


cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eye_cascade;
cv::CascadeClassifier eye_cascade_EYE;


// named pipe variables
static int fd_fifo = -1;
static const char myfifo[] = "/tmp/blinkDfifo";

// Debugging
static const bool kPlotVectorField = false;

// Size constants
static const int kEyePercentTop = 25;
static const int kEyePercentSide = 13;
static const int kEyePercentHeight = 30;
static const int kEyePercentWidth = 35;

// Preprocessing
static const bool kSmoothFaceImage = false;
static const float kSmoothFaceFactor = 0.005;

// Algorithm Parameters
static const int kFastEyeWidth = 50;
static const int kWeightBlurSize = 5;
static const bool kEnableWeight = true;
static const float kWeightDivisor = 1.0;
static const double kGradientThreshold = 50.0;

// Postprocessing
static const bool kEnablePostProcess = true;
static const float kPostProcessThreshold = 0.97;

// Eye Corner
static const bool kEnableEyeCorner = false;

/************************** Namespaces ********************************/

using namespace std;
using namespace cv;

/*********************** Function Definitions *************************/




 
/*
** blinkDetect_task
**
** Description
**  Main function of the blink detect module. This module calls 
**  the other functions required to detect the blink of the eye.
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

//void *blinkDetect_task(void *arg)
int blinkDetect_task()
{
	// welcome message
	cout << "blinkdetect: Task Started " << endl;
	
	
   

	cout << "blinkdetect: loading cascade classifiers " << endl;

	// Load the cascade classifiers
	// Make sure you point the XML files to the right path, or 
	// just copy the files from [OPENCV_DIR]/data/haarcascades directory
	face_cascade.load("xml/haarcascade_frontalface_alt2.xml");
	
	//eye_cascade.load("xml/haarcascade_eye.xml");
	eye_cascade_EYE.load("/usr/local/share/OpenCV/haarcascades/haarcascade_eye.xml");
	//eye_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_lefteye_2splits.xml");
	eye_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_righteye_2splits.xml");
	
	
	
	
    // Open webcam
    cv::Mat image_cam;
    raspicam::RaspiCam_Cv Camera;
    //Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 ); 
    
    // Do Color
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 ); 
    
    
    
    
    
    cout<<"blinkdetect: Opening Camera..."<<endl;
    int cam_open = Camera.open();
    if (!cam_open) 
    {
		cerr<<"blinkdetect: Error opening the camera"<<endl;
		return 0;
	}
	if (face_cascade.empty() || eye_cascade.empty() || eye_cascade_EYE.empty())
	{
		cerr<< "blinkdetect: Error in template load." << endl;
		return 0;
	}
    

	cv::Mat eye_tpl;
	cv::Rect eye_bb;
	
	Size size(320,240);
	cv::Mat image;

	
	cout << "blinkdetect: Let's begin!" << endl;
	

	while (cv::waitKey(15) != 'q')
	{
		Camera.grab();
		Camera.retrieve (image_cam);
		if (image_cam.empty())
		{
			cout << "blinkdetect: ERROR -- No image read from camera -- terminating" << endl;
			break;
		}
			
		// Resizing the image to a smaller size
		resize(image_cam, image, size); 

		// Convert to grayscale and 
		// adjust the image contrast using histogram equalization
		cv::Mat grayTemp;
		cv::Mat gray;
		
		cv::cvtColor(image, grayTemp, CV_BGR2GRAY);
		cv::equalizeHist(grayTemp, gray);
		
		
		//////
		std::vector<cv::Mat> channels; 
		cv::Mat img_hist_equalized;
		cv::cvtColor(image, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format
		cv::split(img_hist_equalized,channels); //split the image into channels
		cv::equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
		cv::merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image
		cv::cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)
		///////// 
				
		
				
		// Find the eyes
		detectEye(image, gray, eye_tpl, eye_bb);
		detectEye(img_hist_equalized, gray, eye_tpl, eye_bb);		
	
	}


	return 0;
}




 
 
/*
** detectEye
**
** Description
**  Finds the face on an image frame using face cascade
**
** Input Arguments:
**  im    The source image
**  tpl   Will be filled with the face template, if detection is successful
**  rect  Will be filled with the face bounding box, will be updated with the new location of the face
**
** Output Arguments:
**  rect  The face bounding box, will be updated with the new location of the face
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
double detectEye(cv::Mat& imColor, cv::Mat& im, cv::Mat& tpl, cv::Rect& rect)
{
	bool ret1 = false, ret2 = false;
	static int counter = 1;
	double sum = 0;	
	std::vector<cv::Rect> faces;
	face_cascade.detectMultiScale(im, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

	// Find the face first, then look for the eyes
	//for (unsigned int i = 0; i < faces.size(); i++)
	if (faces.size() > 0)
	{
		//sum = findEyes_contours(im, faces[0]);
		ret1 = findEyes_classifier(imColor, im, faces[0]);	
		//ret2 = findEyes_hybrid(im, faces[0]);	
		
	}

	return sum;
}





/*
** findEyes_contours
**
** Description
**  Find eyes based on a face image. Code based on code from:
**       http://thume.ca/projects/2012/11/04/simple-accurate-eye-center-tracking-in-opencv/,
**       https://github.com/trishume/eyeLike/blob/master/src/main.cpp
**
** Input Arguments:
**  frame_gray    	grayscale image from the camera
**  Rect			rectangle indicating location of face
**  
**
** Output Arguments:
**  None
**
** Function Return:
**  1
**
** Special Considerations:
**  None
**
**/
double findEyes_contours(cv::Mat frame_gray, cv::Rect face) 
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
						 
	//cv::Rect rightEyeRegion(face.width - eye_region_width - face.width*(kEyePercentSide/100.0),eye_region_top,eye_region_width,eye_region_height);
	
	//cv::rectangle(faceROI, leftEyeRegion, CV_RGB(0,255,0));
	
	cv::Mat eyeL = faceROI(leftEyeRegion);
	//cv::imshow("Left Eye", eyeL);	
	
	cv::Mat eyeBin, eyeSmooth;
	cv::GaussianBlur(faceROI(leftEyeRegion),eyeSmooth,cv::Size(5,5),1.5);	
	cv::threshold(eyeSmooth, eyeBin, 67, 255, CV_THRESH_BINARY);
	
	
	std::vector<std::vector<cv::Point> > contours;
	cv::Mat contourOutput = eyeBin.clone();
	cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    //Draw the contours
    //cv::Mat contourImage(eyeBin.size(), CV_8UC1, cv::Scalar(0,0,0));
    cv::Mat contourImage = eyeL.clone();
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);
    for (size_t idx = 0; idx < contours.size(); idx++) 
    {
        cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
    }
	cv::imshow("Contours", contourImage);		
	
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
	
	
	return 1; 
}







/*
** findEyes_classifier
**
** Description
**  Find eyes based on a face image using cascade classifiers for eyes
**
** Input Arguments:
**  frame_gray    	grayscale image from the camera
**  Rect			rectangle indicating location of face
**  
**
** Output Arguments:
**  None
**
** Function Return:
**  1
**
** Special Considerations:
**  None
**
**/
bool findEyes_classifier(cv::Mat frame_color, cv::Mat frame_gray, cv::Rect face) 
{
	bool eyeDetected = false;
	
	cv::Mat faceROI = frame_gray(face);
	cv::Mat faceROIColor = frame_color(face);
	//cv::imshow("face", faceROI);	

	
	std::vector<cv::Rect> eyes;
	try
	{
		eye_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE); //, cv::Size(5,5));
		if (eyes.size() > 0)
		{
			//cv::rectangle(faceROIColor, eyes[0], CV_RGB(0,255,0));
			//cv::imshow("Left Eye", faceROIColor);
			
			cv::rectangle(frame_color, face, CV_RGB(255,0,0));
			eyes[0].x = eyes[0].x + face.x;
			eyes[0].y = eyes[0].y + face.y;
			cv::rectangle(frame_color, eyes[0], CV_RGB(0,255,0));
			cv::imshow("Left Eye", frame_color);
			
			//cv::Mat eyeDetected = faceROI(eyes[0]);
			//cv::imshow("Left Eye", eyeDetected);
			//cv::waitKey(5);
			eyeDetected = true;
		}
		else
		{
			eyeDetected = false;
		}
	}
	catch( cv::Exception& e )
	{
		const char* err_msg = e.what();
		std::cout << "exception caught: " << err_msg << std::endl;
	}
	
	return eyeDetected;
}




/*
** findEyes_hybrid
**
** Description
**  Find eyes based on a face image using cascade classifiers and
**  relative measurements for eyes
**
** Input Arguments:
**  frame_gray    	grayscale image from the camera
**  Rect			rectangle indicating location of face
**  
**
** Output Arguments:
**  None
**
** Function Return:
**  1
**
** Special Considerations:
**  None
**
**/
bool findEyes_hybrid(cv::Mat frame_gray, cv::Rect face) 
{
	
	// Size constants
	static const int eyeTop = 25;
	static const int eyeSide = 13;
	static const int eyeHeight = 30;
	static const int eyeWidth = 35;
	bool eyeDetected = false;
	cv::Mat faceROI = frame_gray(face);
	
	
	//-- Find eye regions and draw them
	int eye_region_width = face.width * (eyeWidth/100.0);
	int eye_region_height = face.width * (eyeHeight/100.0);
	int eye_region_top = face.height * (eyeTop/100.0);
	
	cv::Rect leftEyeRegion(face.width*(eyeSide/100.0),eye_region_top,eye_region_width,eye_region_height);
	//cv::Rect rightEyeRegion(face.width - eye_region_width - face.width*(eyeSide/100.0),eye_region_top,eye_region_width,eye_region_height);
	
	cv::rectangle(faceROI, leftEyeRegion, CV_RGB(0,255,0));
	
	//cv::Mat eyeL = faceROI(leftEyeRegion);
	
	// Equalize the image before running the classifier
	cv::Mat eyeL;
	cv::equalizeHist(faceROI(leftEyeRegion), eyeL);
	
	// Display the eye
	//cv::imshow("Left Eye", eyeL);	

	std::vector<cv::Rect> eyes;
	try
	{
		eye_cascade_EYE.detectMultiScale(eyeL, eyes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE); //, cv::Size(5,5));
		if (eyes.size() > 0)
		{
			//cv::Mat eyeDetected = eyeL(eyes[0]);
			//cv::imshow("Left Eye", eyeDetected);
			//cerr << "size: "<< eyeDetected.size() << endl;
			eyeDetected = true;
		}
		else
		{
			eyeDetected = false;
		}
	}
	catch( cv::Exception& e )
	{
		const char* err_msg = e.what();
		std::cout << "exception caught: " << err_msg << std::endl;
	}
	
	return eyeDetected;
}



////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


/*
** findEyes_ORIGINAL
**
** Description
**  Find eyes based on a face image. Code based on code from:
**       http://thume.ca/projects/2012/11/04/simple-accurate-eye-center-tracking-in-opencv/,
**       https://github.com/trishume/eyeLike/blob/master/src/main.cpp
**
** Input Arguments:
**  frame_gray    	grayscale image from the camera
**  Rect			rectangle indicating location of face
**  
**
** Output Arguments:
**  None
**
** Function Return:
**  1
**
** Special Considerations:
**  None
**
**/
double findEyes_ORIGINAL(cv::Mat frame_gray, cv::Rect face) 
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
						 
	//cv::Rect rightEyeRegion(face.width - eye_region_width - face.width*(kEyePercentSide/100.0),
	//					  eye_region_top,eye_region_width,eye_region_height);
	
	//cv::rectangle(faceROI, leftEyeRegion, CV_RGB(0,255,0));
	
	cv::Mat eyeL = faceROI(leftEyeRegion);
	//cv::imshow("Left Eye", eyeL);	

	
	std::vector<cv::Rect> eyes;
	try
	{
		eye_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE); //, cv::Size(5,5));
		if (eyes.size() > 0)
		{
			cv::Mat eyeDetected = faceROI(eyes[0]);
			cv::imshow("Left Eye", eyeDetected);
			//cv::waitKey(5);
		}
	}
	catch( cv::Exception& e )
	{
		const char* err_msg = e.what();
		std::cout << "exception caught: " << err_msg << std::endl;
	}
	
	return 1;
//////////////////////////////////////////////////	
	
	
	
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




/*
** trackEye
**
** Description
**  Perform template matching to search the user's eye in the given image.
**
** Input Arguments:
**  im    The source image
**  tpl   The eye template
**  rect  The eye bounding box, will be updated with the new location of the eye
**
** Output Arguments:
**  rect  The eye bounding box, will be updated with the new location of the eye
**
** Function Return:
**  None
**
** Special Considerations:
**  None
**
**/
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
