// Cristóbal Carnero Liñán <grendel.ccl@gmail.com>

#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cvblob.h>
#include "red_object_tracking.h"
#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#include <imgproc.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <semaphore.h>
#include "estructuras.h"

using namespace cvb;
using namespace cv;

bool isCircle(std::string window, cv::Mat image, int x1, int y1, int ancho,
		int alto) {

	return true;

	vector<Vec3f> circulos;
	cv::Mat gris;
	// creo una subimagen de las coordenadas que me pasan del tracking
	cv::Mat subImage(image, cvRect(x1, y1, ancho, alto));

	medianBlur(subImage, gris, 5);
	cvtColor(subImage, gris, CV_BGR2GRAY);
	Canny(subImage, subImage, 35, 90);

	// busco los circulos en la imagen gris
	HoughCircles(gris, circulos, CV_HOUGH_GRADIENT, 1, gris.rows / 4, 100, 100,
			0, 0);

	imshow(window, gris);

	// libero los recursos
	subImage.release();
	gris.release();

	// si encuentro UN solo circulo, devuelvo TRUE. Sino significa que tengo mal hecho el analisis.
	if (circulos.size() == 1) {
		return true;
	}
	return false;
}
void *startRedObjectTracking(void *arg) {
	ThreadAttr *args = (ThreadAttr *) arg;

	//int startRedObjectTracking() {

//int main() {
	printf("\r\n----------------------------------------\r\n");
	printf("\t\tPID: %d", getpid());
	printf("\r\n----------------------------------------\r\n");

	CvTracks tracks;

//	cvNamedWindow("red_object_tracking", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("red_ball", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("isCircle", CV_WINDOW_AUTOSIZE);

	CvCapture *capture = cvCaptureFromCAM(0);
	cvGrabFrame(capture);
	IplImage *img = cvRetrieveFrame(capture);

	CvSize imgSize = cvGetSize(img);

	sem_wait(&args->mutex);
	args->data.imgSize.height = imgSize.height;
	args->data.imgSize.width = imgSize.width;
	sem_post(&args->mutex);

	IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

	IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1,
			CV_SHAPE_RECT, NULL);

	//para el calculo de la distacia
	float pendiente = (DIAM_CERCA - DIAM_LEJOS) / (DIST_CERCA - DIST_LEJOS);
	float dominio = DIAM_CERCA - pendiente * DIST_CERCA;
	//****************

	bool quit = false;
	while (!quit && cvGrabFrame(capture)) {
		IplImage *img = cvRetrieveFrame(capture);

		cvConvertScale(img, frame, 1, 0);

		IplImage *segmentated = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);

		// crea una matriz (IplImage* segmented) solo de pixeles blancos (para los rojos) y negros (para el resto)
		for (int j = 0; j < imgSize.height; j++)
			for (int i = 0; i < imgSize.width; i++) {
				CvScalar c = cvGet2D(frame, j, i);

				double b = ((double) c.val[0]) / 255.;
				double g = ((double) c.val[1]) / 255.;
				double r = ((double) c.val[2]) / 255.;
				unsigned char f = 255 * ((r > 0.2 + g) && (r > 0.2 + b));

				cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
			}

		// pagina 120 de O'Reilly Learning Opencv
		cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN,
				1);

		IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL,
				1);

		// Empieza a trabajar con los BLOb
		CvBlobs blobs;

		// etiqueta las partes interesantes del blob
		cvLabel(segmentated, labelImg, blobs);
		// aquellos blobs que no estan dentro del rango de areas (500-1000000), se borran
		cvFilterByArea(blobs, 500, 1000000);
		// dibujo los blobs en frame
		cvRenderBlobs(labelImg, blobs, frame, frame,
				CV_BLOB_RENDER_BOUNDING_BOX | CV_BLOB_RENDER_CENTROID);
		// actualizo los tracks
		cvUpdateTracks(blobs, tracks, 200., 5);
		// dibujo los tracks
		cvRenderTracks(tracks, frame, frame,
				CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);

		// paso frame a un Mat
		cv::Mat frame2(frame);
		for (CvTracks::const_iterator itTracks = tracks.begin();
				itTracks != tracks.end(); ++itTracks) {
			double ancho = itTracks->second->maxx - itTracks->second->minx;
			double alto = itTracks->second->maxy - itTracks->second->miny;
			double diametro = sqrt(pow(ancho, 2) + pow(alto, 2));
			double radio = diametro / 2.5;

			// si es un circulo imprimo las coordenadas
			if (isCircle("isCircle", frame, itTracks->second->minx,
					itTracks->second->miny, ancho, alto)) {
				printf("Es un circulo=> centro {x: %f, y: %f} radio: %f \r\n",
						itTracks->second->centroid.x,
						itTracks->second->centroid.y, radio);

				sem_wait(&args->mutex);
				args->data.tPos.x = itTracks->second->centroid.x;
				args->data.tPos.y = itTracks->second->centroid.y;
				args->data.tPos.z = pendiente * diametro + dominio; // se calcula la distancia en base a los valores predefinidos
				args->data.tPos.diametro = diametro;
				sem_post(&args->mutex);
			}

			// dibujo un circulo sobre lo que estoy trackeando
			circle(frame2,
					cvPoint(itTracks->second->centroid.x,
							itTracks->second->centroid.y), radio,
					cvScalar(255, 255, 0), 3, 8, 0);
		}
		imshow("red_ball", frame2);

		// libero la memoria usada para los label y la imagen 0/1
		cvReleaseImage(&labelImg);
		cvReleaseImage(&segmentated);

		char k = cvWaitKey(10) & 0xff;
		switch (k) {
		case 27:
		case 'q':
		case 'Q':
			quit = true;
			break;
		}

		// libero los blobs
		cvReleaseBlobs(blobs);
	}

	cvReleaseStructuringElement(&morphKernel);
	cvReleaseImage(&frame);

	cvDestroyWindow("red_ball");
	cvDestroyWindow("isCircle");

	return 0;
}

