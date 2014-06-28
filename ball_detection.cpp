/*****************************************************************************************
 *  Name    : Fast object tracking using the OpenCV library                               *
 *  Author  : Lior Chen <chen.lior@gmail.com>                                             *
 *  Notice  : Copyright (c) Jun 2010, Lior Chen, All Rights Reserved                      *
 *          :                                                                             *
 *  Site    : http://www.lirtex.com                                                       *
 *  WebPage : http://www.lirtex.com/robotics/fast-object-tracking-robot-computer-vision   *
 *          :                                                                             *
 *  Version : 1.0                                                                         *
 *  Notes   : By default this code will open the first connected camera.                  *
 *          : In order to change to another camera, change                                *
 *          : CvCapture* capture = cvCaptureFromCAM( 0 ); to 1,2,3, etc.                  *
 *          : Also, the code is currently configured to tracking RED objects.             *
 *          : This can be changed by changing the hsv_min and hsv_max vectors             *
 *          :                                                                             *
 *  License : This program is free software: you can redistribute it and/or modify        *
 *          : it under the terms of the GNU General Public License as published by        *
 *          : the Free Software Foundation, either version 3 of the License, or           *
 *          : (at your option) any later version.                                         *
 *          :                                                                             *
 *          : This program is distributed in the hope that it will be useful,             *
 *          : but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 *          : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 *          : GNU General Public License for more details.                                *
 *          :                                                                             *
 *          : You should have received a copy of the GNU General Public License           *
 *          : along with this program.  If not, see <http://www.gnu.org/licenses/>        *
 ******************************************************************************************/
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include "Circle.h"
#include <libcalg-1.0/libcalg.h>
#include <libcalg-1.0/libcalg/list.h>
#include <list>
#include "ball_detection.h"

//aca quiero agregar al array estados los circulos
//el circulo mas viejo se pierde como veras
int count = FRAMES;
void addCircle2(CvSeq estados[FRAMES], CvSeq circles) {
	for (int i= 0; i < FRAMES - 1;i++)
	{
		estados[i] = estados[i + 1];
	}
	estados[FRAMES - 1] = circles;

	if (count != 0)
		count--;

}

void addCircle(std::vector<Circle> estados[FRAMES], std::vector<Circle> circles) {

	for (int i= 0; i < FRAMES - 1;i++)
	{
		estados[i] = estados[i + 1];
	}
	estados[FRAMES - 1] = circles;

	if (count != 0)
		count--;

}



int startBallDetection() {
	//Default capture size - 640x480
	CvSize size = cvSize(720, 480);
	// Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
	CvCapture* capture = cvCaptureFromCAM(1);
	if (!capture) {
		fprintf(stderr, "ERROR: capture is NULL \n");
		getchar();
		return -1;
	}
	// Create a window in which the captured images will be presented
	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("EdgeDetection", CV_WINDOW_AUTOSIZE);
	// Detect a red ball
	CvScalar hsv_min = cvScalar(150, 84, 130, 0);
	CvScalar hsv_max = cvScalar(358, 256, 255, 0);
	IplImage * hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);
	IplImage* thresholded = cvCreateImage(size, IPL_DEPTH_8U, 1);

	//ESTA ES LA VARIABLE QUE TENGO QUE PASAR COMO PARAMETRO A TODAS LAS FUNCIONES
	//UN ARRAY DE CcSeq y quiero que sean punteros a CvSeq

	//CvSeq estados[FRAMES];
	std::vector<Circle> estados[FRAMES];

//	ArrayList estados2[FRAMES];
//	ArrayList* miArray = arraylist_new(sizeof(Circle));

	//en esta funcion queiro inicializar la variable que paso como parametro en null
//	inicializar(estados);
	while (1) {
		// Get one frame
		IplImage* frame = cvQueryFrame(capture);
		if (!frame) {
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}
		// Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
		cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
		// Filter out colors which are out of range.
		cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
		// Memory for hough circles
		CvMemStorage* storage = cvCreateMemStorage(0);
		// hough detector works better with some smoothing of the image
		cvSmooth(thresholded, thresholded, CV_GAUSSIAN, 9, 9);
		CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT,
				2, thresholded->height / 4, 100, 50, 10, 400);




		std::vector<Circle> circulos;

		for (int i = 0; i < circles->total; i++) {

			float* p = (float*) cvGetSeqElem(circles, i);

			Circle a = Circle(cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]));
		//	ArrayListValue mucosa;
			//mucosa = a;
			circulos.push_back(a);
		//	arraylist_append(miArray, &a);
			//   circulos.add(a);

			printf("Ball! x=%f y=%f r=%f\n\r", p[0], p[1], p[2]);
			cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
					cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0 );

		}
		addCircle( estados, circulos);
		//     estados.add(circulos);
	//	addCircle( estados2, *miArray);

		Circle *a = analizar( estados);
		if (a != NULL && a->getRadio() > 0 && a->getCentro().x > 0&& a->getCentro().y > 0 )
			cvCircle(frame, a->getCentro(), a->getRadio(), CV_RGB(255,0,0), 3, 8, 0);

		cvShowImage("Camera", frame); // Original stream with detected ball overlay
		cvShowImage("HSV", hsv_frame); // Original stream in the HSV color space
		cvShowImage("After Color Filtering", thresholded); // The stream after color filtering
		cvReleaseMemStorage(&storage);
		// Do not release the frame!
		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
		//remove higher bits using AND operator
		if ((cvWaitKey(10) & 255) == 27)
			break;
	}
	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvDestroyWindow("mywindow");
	return 0;
}

Circle *analizar(std::vector<Circle> estados[FRAMES]) {
	Circle *cercano;
	Circle *predecida = new Circle(cvPoint(0, 0), 0);
	Circle *promedio = new Circle(cvPoint(0, 0), 0);
	int i = count;
//	while (i < 5 && estados[i] == NULL)
//		i++;
	if (i < FRAMES) {
		int e = 0;


		while(i < FRAMES && estados[i].size() == 0)
			i++;
		if (i < FRAMES)
		{
			 //list<int>::iterator i;
			//for(i=L.begin(); i != L.end(); ++i)



			Circle anterior = estados[i][0];
			Circle anterior2 = anterior;
			CvPoint centro;
			for (int n = i; n < FRAMES - 1; n++) {

				cercano = buscarCercano(estados[n + 1], anterior);
				if (cercano != NULL)
				{
					centro = predecida->getCentro();
					centro.x += cercano->getCentro().x - anterior.getCentro().x;
					centro.y += cercano->getCentro().y - anterior.getCentro().y;

					predecida->setCentro(centro);
				//	predecida->getCentro().x =+ cercano.getCentro().x - anterior.getCentro().x;
				//	predecida->getCentro().y =+ cercano.getCentro().y - anterior.getCentro().y;




					centro = promedio->getCentro();
					centro.x += cercano->getCentro().x;
					centro.y += cercano->getCentro().y;
					promedio->setCentro(centro);
					promedio->setRadio(promedio->getRadio() + cercano->getRadio());

					anterior2 = anterior;
					anterior = *cercano;

					e++;
				}
				else
				{
					e = e+1;
					e--;
				}
			}
			if (e != 0) {

				centro = anterior2.getCentro();
				centro.x += predecida->getCentro().x / e;
				centro.y += predecida->getCentro().y / e;

				centro.x += anterior.getCentro().x;
				centro.y += anterior.getCentro().y;

				centro.x = centro.x / 2;
				centro.y  = centro.y / 2;

			//	anterior.getCentro().x += predecida->getCentro().x / e;
			//	anterior.getCentro().y += predecida->getCentro().y / e;

				anterior.setCentro(centro);
				promedio= &anterior;
				return promedio;
				promedio->setCentro(centro);
				promedio->setCentro(centro);
				promedio->setRadio(promedio->getRadio() / e);
			}
			else
			{
				promedio = &anterior;
			}
		}
		else
			return NULL;

	}
	return promedio;
}

Circle *buscarCercano(std::vector<Circle> circulos, Circle c) {
	Circle *cercano = NULL;

	int distanciaCercano = 0;
	for (int i = 0; i < ((int)circulos.size()); i++) {
	//	float* p = (float*) cvGetSeqElem(circulos, i);
		Circle *c2 = &circulos[i];
		int dist = distancia(c, *c2);

		 if (i == 0)
		{
			distanciaCercano = dist;
			cercano = c2;
		}
		else if (dist < distanciaCercano) {
			cercano = c2;
			distanciaCercano = dist;
		}
	}

	return cercano;

}

int distancia(Circle a, Circle b) {

	return sqrt(
			pow(a.getCentro().x - b.getCentro().x, 2)
					+ pow(a.getCentro().y - b.getCentro().y, 2));
}
Circle toCircle(float *p) {
	return Circle(cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]));

}
void inicializar(CvSeq *estados[]) {

	memset(*estados,0, sizeof(CvSeq) * FRAMES);
}


