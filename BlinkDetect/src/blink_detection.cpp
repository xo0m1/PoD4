// Blink Detection.cpp : Defines the entry point for the console application.
//



// OpenCV Sample Application: facedetect.c

// Include header files

#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#include <iostream>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <raspicam/raspicamtypes.h>
#include <raspicam/raspicam_cv.h>
#include "../include/blink_detection.h"

// Create memory for calculations
static CvMemStorage* storage = 0;

// Create a new Haar classifier
static CvHaarClassifierCascade* cascade = 0;

// Function prototypes


static void detect_and_draw(); 
static void CopySubImage(IplImage *imageSource, IplImage *imageDest, int xorigin, int yorigin, int width, int height);

// Create a string that contains the cascade name
//const char* CASCADE_FILE = "Cascade Files/haarcascade_frontalface_alt.xml";
const char* CASCADE_FILE  = "/home/pi/openCV_others/haarcascade_mcs_eyepair_big.xml";

//DWORD tickCountPrev = 0;
static int frames = 0;
static float fps = 0;

// Images to capture the frame from video or camera or from file
static IplImage        *frame, 
                        *frame_copy = 0, 
                        *frame_eyes = 0, 
                        *frame_eyes_prev = 0, 
                        *frame_eyes_diff = 0,
                        *frame_eyes_gray = 0,
                        *frame_eyes_smooth = 0,
                        *frame_eyes_bin = 0;
                        
using namespace std;

// Main function, defines the entry point for the program.
int blink_detection( void )
{
    // Structure for getting video from camera or avi
    CvCapture* capture = 0;

    // Load the HaarClassifierCascade
    cascade = (CvHaarClassifierCascade*)cvLoad( CASCADE_FILE, 0, 0, 0 );
    cout<<"cascade file: "<< CASCADE_FILE<< endl;
    
    // Check whether the cascade has loaded successfully. Else report and error and quit
    if( !cascade )
    {
        fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
        return -1;
    }
    
    // Allocate the memory storage
    storage = cvCreateMemStorage(0);

	// Setup the camera capture
	//capture = cvCaptureFromCAM(0);
	//cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 30);
	
	
	cv::Mat frame_mat; // CONVERT TO IPLIMAGE
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
	Camera.set(CV_CAP_PROP_EXPOSURE, raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW); //-1 is auto, values range from 0 to 100  

	// Create a new named window with title: result
	cvNamedWindow( "result", 1 );
	//cvNamedWindow( "eyes", 1);
	//cvNamedWindow( "eyes_diff", 1);
	cvNamedWindow("eyes_gray", 1);
	cvNamedWindow("eyes_smooth", 1);
	cvNamedWindow( "eyes_bin", 1);

    // Find if the capture is loaded successfully or not.

    // If loaded succesfully, then:
    if( 1 )
    //if( capture )
    {
        // Capture from the camera.
        for(;;)
        {
			
			Camera.grab();
			Camera.retrieve(image_cam);
			if (image_cam.empty())
			{
				cerr << "no image read from camera -- terminating" << endl;
				break;
			}
			
			// Resizing the image to a smaller size
			cv::Size size(320,240);
			cv::resize(image_cam, frame_mat, size); 
			
			// Copy to iimage structure			
			//frame = cvCreateImage(cvSize(frame_mat.cols,frame_mat.rows),8,3);
			//IplImage ipltemp=frame_mat;			
			//cvCopy(&ipltemp,frame);
			
			IplImage copy = frame_mat;
			frame = &copy;
		
		
            // Capture the frame and load it in IplImage
            //if( !cvGrabFrame( capture ))
                //break;
            //frame = cvRetrieveFrame( capture );

            //// If the frame does not exist, quit the loop
            //if( !frame )
                //break;
            
            // Allocate framecopy as the same size of the frame
            if( !frame_copy )
            {
                frame_copy = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );
                                frame_eyes = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );
                                frame_eyes_prev = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, frame->nChannels );
                                frame_eyes_diff = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, frame->nChannels );
                                frame_eyes_gray = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1 );
                                frame_eyes_smooth = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1 );
                                frame_eyes_bin = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1 );
            }

            // Check the origin of image. If top left, copy the image frame to frame_copy. 
            if( frame->origin == IPL_ORIGIN_TL )
                cvCopy( frame, frame_copy, 0 );
            // Else flip and copy the image
            else
                cvFlip( frame, frame_copy, 0 );
            
            // Call the function to detect and draw the face
            detect_and_draw();

			frames++;
			/*
			DWORD tickCount = GetTickCount();
			if (tickCount > tickCountPrev + 1000) 
			{
					fps = frames * 1000.0f / (tickCount - tickCountPrev);
					frames = 0;
					tickCountPrev = tickCount;      
			}
			* */

			char strFPS[50];
			//char strINT[50];

			sprintf(strFPS, "FPS: %f", fps);
			//sprintf_s(strINT, "INT: %f", imgDifference.val[0]);

			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.75, 0.75, 0, 2, CV_AA);

			CvSize textSize;
			cvGetTextSize("A", &font, &textSize, 0);

			cvPutText(frame_copy, strFPS, cvPoint(0, 1 * textSize.height), &font, CV_RGB(0, 0, 255));
			//cvPutText(frame_copy, strINT, cvPoint(0, 2 * textSize.height), &font, CV_RGB(0, 0, 255));

			// Show the image in the window named "result"
			cvShowImage( "result", frame_copy );
			//cvShowImage( "eyes", frame_eyes );
			//cvShowImage( "eyes_diff", frame_eyes_diff );
			cvShowImage( "eyes_gray", frame_eyes_gray );
			cvShowImage( "eyes_smooth", frame_eyes_smooth );
			cvShowImage( "eyes_bin", frame_eyes_bin );
			
			//cerr << "here" << endl;

            // Wait for a while before proceeding to the next frame
            //if( cvWaitKey( 10 ) >= 0 )
            if (cv::waitKey(15) == 'q')
            {
                break;
			}
        }

        // Release the images, and capture memory
        cvReleaseImage( &frame_copy );
                cvReleaseImage( &frame_eyes );
                cvReleaseImage( &frame_eyes_prev );
                cvReleaseImage( &frame_eyes_diff );
                cvReleaseImage( &frame_eyes_bin );
        cvReleaseCapture( &capture );
    }
    
    // Destroy the window previously created with filename: "result"
    cvDestroyWindow("result");
        //cvDestroyWindow("eyes");
        //cvDestroyWindow("eyes_diff");
        cvDestroyWindow("eyes_gray");
        cvDestroyWindow("eyes_smooth");
        cvDestroyWindow("eyes_bin");

    // return 0 to indicate successfull execution of the program
    return 0;
}

// Function to detect and draw any faces that is present in an image
void detect_and_draw()
{
    int scale = 1;

    // Create a new image based on the input image
    //IplImage* temp = cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 3 );

    // Create two points to represent the face locations
    CvPoint pt1, pt2;
    //int i;
	double avgIntensityValue = 0.0f;
	CvScalar imgDifference;
	imgDifference.val[0] = 0;

    // Clear the memory storage which was used before
    cvClearMemStorage( storage );

    // Find whether the cascade is loaded, to find the faces. If yes, then:
    if( cascade )
    {

        // There can be more than one face in an image. So create a growable sequence of faces.
        // Detect the objects and store them in the sequence
        CvSeq* faces = cvHaarDetectObjects( frame_copy, cascade, storage,
                                            1.1, 2, CV_HAAR_DO_CANNY_PRUNING,
                                            cvSize(50, 50) );

        // Loop the number of faces found.
        //for( i = 0; i < (faces ? faces->total : 0); i++ )
        if((faces ? faces->total : 0) > 0)
        {
           // Create a new rectangle for drawing the face
            //CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
			CvRect* r = (CvRect*)cvGetSeqElem( faces, 0 );

			//cvSet(frame_eyes, cvScalar(0,0,0));
			//CopySubImage(frame_copy, frame_eyes, r->x, r->y, r->width, r->height);

			cvAbsDiff(frame_copy, frame_eyes_prev, frame_eyes_diff);
			cvCopy(frame_copy, frame_eyes_prev);

			//cvCvtColor(frame_eyes_diff, frame_eyes_gray, CV_RGB2GRAY);
			//cvSmooth(frame_eyes_gray, frame_eyes_smooth);			
			cvSmooth(frame_eyes_diff, frame_eyes_smooth);
			cvThreshold(frame_eyes_smooth, frame_eyes_bin, 64, 255, CV_THRESH_BINARY);

            //imgDifference = cvSum(frame_eyes_diff);

            // Find the dimensions of the face,and scale it if necessary
            pt1.x = r->x*scale;
            pt2.x = (r->x+r->width)*scale;
            pt1.y = r->y*scale;
            pt2.y = (r->y+r->height)*scale;

            // Draw the rectangle in the input image
            cvRectangle( frame_copy, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
                        
                        /*cvSetImageROI(img, *r);
                        avgIntensity = cvAvg(img);
                        avgIntensityValue = avgIntensity.val[0] * 0.3 + avgIntensity.val[1] * 0.59 + avgIntensity.val[2] * 0.11;
                        cvResetImageROI(img);*/
        }
    }
}

void CopySubImage(IplImage *imageSource, IplImage *imageDest, int xorigin, int yorigin, int width, int height)
{
        CvRect roi;

        roi.x = xorigin;
        roi.y = yorigin;
        roi.width = width;
        roi.height = height;

        cvSetImageROI(imageSource, roi);
        cvSetImageROI(imageDest, roi);

        //imageDest = cvCreateImage( cvSize(roi.width, roi.height), imageSource->depth, imageSource->nChannels );
        
        cvCopy(imageSource, imageDest);
        
        cvResetImageROI(imageSource);
        cvResetImageROI(imageDest);
}
