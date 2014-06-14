#include <iostream>
/*#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
*/

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

#include "contours.h"
#include "opencv2/opencv.hpp"
/*
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
*/
/*
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <libcalg-1.0/libcalg.h>
#include <libcalg-1.0/libcalg/list.h>
*/


void drawAllTriangles(Mat& img, const vector<vector<Point> >& contours)
{
	vector<Point> approxTriangle;
	for (size_t i = 0; i < contours.size(); i++)
	{
		approxPolyDP(contours[i], approxTriangle,
				arcLength(Mat(contours[i]), true) * 0.02, true);
		if (approxTriangle.size() == 3)
		{
			drawContours(img, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
			vector<Point>::iterator vertex;
			for (vertex = approxTriangle.begin();
					vertex != approxTriangle.end(); ++vertex)
			{
				circle(img, *vertex, 3, Scalar(0, 0, 255), 1);
			}
		}
	}
}

int searchContours() {

//	CvSize size = cvSize(640, 480);
	// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.

	cvNamedWindow("InputImage", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Contours", CV_WINDOW_AUTOSIZE);

	CvCapture* capture = cvCaptureFromCAM(0);
	if (!capture) {
	//	fprintf(stderr, "ERROR: capture is NULL \n");
	//	getchar();
		return -1;
	}

	cvGrabFrame(capture);
	IplImage *img = cvRetrieveFrame(capture);
	CvSize size = cvGetSize(img);

//	IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

	// Detect a red ball
	CvScalar hsv_min = cvScalar(150, 84, 130, 0);
	CvScalar hsv_max = cvScalar(358, 256, 255, 0);

//	IplImage * hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
//	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);

	bool quit = false;
	while (!quit && cvGrabFrame(capture)) {
		IplImage *img = cvRetrieveFrame(capture);

		/*// Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
		cvCvtColor(img, hsv_frame, CV_BGR2HSV);
		// Filter out colors which are out of range.
		::cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);

		cv::Mat image(hsv_frame);
		cv::Mat gaussian(hsv_frame);
*/
		cv::Mat image(img);
	//	GaussianBlur( image, image, Size(0,0), 3,3);



		//Prepare the image for findContours
		cvtColor( image, image, CV_BGR2GRAY);
		image.convertTo(image, CV_8UC1);
		cv::threshold(image, image, 128, 255, CV_THRESH_BINARY);//128-255

		//Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
		std::vector<std::vector<cv::Point> > contours;
		cv::Mat contourOutput = image.clone();
		cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

		//Draw the contours
		cv::Mat contourImage(image.size(), CV_8UC3, cv::Scalar(0,0,0));
		cv::Scalar colors[3];
		colors[0] = cv::Scalar(255, 0, 0);
		colors[1] = cv::Scalar(0, 255, 0);
		colors[2] = cv::Scalar(0, 0, 255);
		for (size_t idx = 0; idx < contours.size(); idx++) {
			cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
		}

		drawAllTriangles(contourImage, contours);




		imshow("InputImage", image);
		cvMoveWindow("InputImage", 0, 0);
		imshow("Contours", contourImage);
		cvMoveWindow("Contours", 800, 0);

		char k = cvWaitKey(10) & 0xff;
		switch (k) {
			case 27:
			case 'q':
			case 'Q':
				quit = true;
				break;
		}
	}
    return 0;
}

