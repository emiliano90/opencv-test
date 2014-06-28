// Cristóbal Carnero Liñán <grendel.ccl@gmail.com>


#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cvblob.h>
#include "camara2_detection.h"
#include <sys/time.h>
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


using namespace cvb;
using namespace cv;

vector<CvScalar> muestras2;
CvScalar hsvMax2;// = (340, 255, 255);
CvScalar hsvMin2;// =(310, 125, 153);
/*muestras[0] = CV_RGB(200, 142, 202);
muestras[1] = CV_RGB(176, 156, 221);
muestras[1] = CV_RGB(182, 72, 151);
muestras[1] = CV_RGB(182, 72, 151);

CvScalar muestra1
CvScalar muestra2 = CV_RGB(176, 156, 221);
CvScalar muestra3 = CV_RGB(182, 72, 151);
CvScalar muestra4 = CV_RGB(182, 72, 151);*/
//rgb(173, 72, 121)
bool isCircle2(std::string window, cv::Mat image, int x1, int y1, int ancho,
		int alto) {

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
void *startAlturaDetection(void *arg){
	ThreadAttr *args = ( ThreadAttr *)arg;

 //int startRedObjectTracking() {


//int main() {
	printf("\r\n----------------------------------------\r\n");
	printf("\t\tPID: %d", (unsigned int)pthread_self());
	printf("\r\n----------------------------------------\r\n");

/*
 //noche
 	muestras.push_back(CV_RGB(200, 142, 202));
	muestras.push_back(CV_RGB(176, 156, 221));
	muestras.push_back(CV_RGB(182, 72, 151));
*/
//dia
//mi cuarto
	/*
	muestras.push_back(CV_RGB(191, 99, 217));
	muestras.push_back(CV_RGB(198, 95, 181));
	muestras.push_back(CV_RGB(194, 108, 186));
	muestras.push_back(CV_RGB(157, 71, 158));
	muestras.push_back(CV_RGB(155, 64, 105));
	muestras.push_back(CV_RGB(102, 37, 90));
	muestras.push_back(CV_RGB(160, 137, 176));//en movimiento
	muestras.push_back(ZCV_RGB(211, 146, 228));
*/
//(179, 255, 255)max
	/*noche
	hsvMax.val[0] = 179;
	hsvMax.val[1] = 255;
	hsvMax.val[2] = 255;
//310, 125, 153
	hsvMin.val[0] = 135;
	hsvMin.val[1] = 56;//76 bien
	hsvMin.val[2] = 130;//125
*/

	hsvMax2.val[0] = 179;
	hsvMax2.val[1] = 255;
	hsvMax2.val[2] = 255;

	hsvMin2.val[0] = 135;
	hsvMin2.val[1] = 66;//76 bien  56
	hsvMin2.val[2] = 140;//125  //130

	muestras2.push_back(CV_RGB(250, 144, 255));


	struct timeval start, end;

	long seconds, useconds, mtime = 0;


	gettimeofday(&start, NULL);


	CvTracks tracks;

//	cvNamedWindow("red_object_tracking", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("redDetection", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("isCircle", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("gray", CV_WINDOW_AUTOSIZE);

//	cvNamedWindow("redMorfhology", CV_WINDOW_AUTOSIZE);

	CvCapture *capture = cvCaptureFromCAM(0);

//	system("v4l2-ctl -d /dev/video1 -i 0 -s 0 --set-fmt-video=width=720,height=480,pixelformat=0");

//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 540);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 360);
	double widht = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	double height = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);

	printf("widht: %f, height: %f",widht, height);
//	double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
//	printf("%f",fps);
    //cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 29);

	cvGrabFrame(capture);
	IplImage *img = cvRetrieveFrame(capture);

	CvSize imgSize = cvGetSize(img);
	cvMoveWindow("redDetection", imgSize.width, 0);
	args->data.imgSize.height = imgSize.height;
	args->data.imgSize.width = imgSize.width;

	IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

	IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1,
			CV_SHAPE_RECT, NULL);

	//esto es para grabar el video en un archivo

	time_t _tm =time(NULL );

	struct tm * curtime = localtime ( &_tm );

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
	std::string str(buffer);

	string source = "/home/toshiba/tesis/video/";
	source.append(buffer);
	source.append(".avi");

//"IYUV

	VideoWriter outputVideo;                                        // Open the output
	outputVideo.open(source, CV_FOURCC('X','V','I','D'), 12, imgSize, false);

	if (!outputVideo.isOpened())
	{
		printf("Could not open the output video for write: %s", source.c_str() );
		return 0;
	}

	//para el calculo de la distacia
	float pendiente =  float(DIST_CERCA - DIST_LEJOS) / float(DIAM_CERCA - DIAM_LEJOS);
	float dominio = DIST_CERCA - pendiente * DIAM_CERCA;
	//****************
	bool bContinue = true;

	int fps = 0;
	bool quit = false;
	while (!quit) {

		gettimeofday(&end, NULL);

		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime += ((seconds) * 1000 + useconds/1000.0) + 0.5;
		fps++;
		if (mtime > 1000)
		{
			printf("Elapsed time: %ld milliseconds, fps: %d \n", mtime, fps);
			fps = 0;
			mtime = 0;
		}

		gettimeofday(&start, NULL);

		IplImage *img = cvQueryFrame(capture);
	//	IplImage *img = cvRetrieveFrame(capture);
	//	cvGrabFrame(capture);

		cvConvertScale(img, frame, 1, 0);

		cvShowImage("camera", frame);

		//cvSize(640,640)
		IplImage *segmentated = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);

		cvCvtColor(frame, frame, COLOR_BGR2HSV);


		// crea una matriz (IplImage* segmented) solo de pixeles blancos (para los rojos) y negros (para el resto)
		for (int j = 0; j < imgSize.height; j++)
			for (int i = 0; i < imgSize.width; i++) {
				CvScalar c = cvGet2D(frame, j, i);

				unsigned char f = 255 * !isSimilarHsv2(c);
			/*	double b = ((double) c.val[0]) / 255.;
				double g = ((double) c.val[1]) / 255.;
				double r = ((double) c.val[2]) / 255.;
				unsigned char f = 255 * !((r > 0.1 + g) && (r > b));
// && (g - 0.1 < b) && (g > b - 0.5)
  				*/

				cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
			}



		// pagina 120 de O'Reilly Learning Opencv
	//	cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN,1);

	//	cv::Mat redMat(segmentated);
	/*	isCircle("isCircle", segmentated, 0,
							0, imgSize.width, imgSize.height);

*/

		// busco los circulos en la imagen gris

//		cvCvtColor(segmentated, segmentated, CV_RGB2GRAY);
	//	Canny(subImage, subImage, 35, 90);
/*
		cvSmooth(segmentated,// function input
				segmentated,// function output
				CV_GAUSSIAN,// use Gaussian filter (average nearby pixels, with closest pixels weighted more)
		        9,// smoothing filter window width
		        9);*/




//		CvMemStorage *storage = cvCreateMemStorage(0);
//		CvSeq* p_seqCircles;
		//IplImage *grayImage = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
		Mat segmented(segmentated);
		Mat grayImage;
		cvtColor(segmented, grayImage, CV_RGB2GRAY);

		imshow("gray", segmented);

		Canny(grayImage, grayImage, 5, 70, 3);
	//	GaussianBlur( grayImage, grayImage, cv::Size(15, 15), 0, 0 );
		GaussianBlur( grayImage, grayImage, cv::Size(3, 3), 0, 0 );
	//	smooth(grayImage, grayImage, CV_GAUSSIAN, 15, 15);

		vector<cv::Vec3f> circles;
		HoughCircles(grayImage, circles, CV_HOUGH_GRADIENT, 2, imgSize.height/4, 50, 10, 1, 20);//300,20

//		float* p_fltXYRadius;

		Posicion pos;
		pos.x = -1;
		pos.y = -1;
		pos.z = -1;
		float dist = -1;

		sem_wait(&args->mutex);

		Posicion predecida = calcular2(args->data.tLastPos);
		for(int i = 0; i < circles.size(); i++){
		//	printf("ball position x: %f  y: %f radio: %f \n", circles[i][0], circles[i][1], circles[i][2]);
		//	printf("center x: %d  y: %d radio: %f \n", imgSize.width / 2, imgSize.height / 2);


			Posicion p;
			p.x = circles[i][0];
			p.y = circles[i][1];
			p.z = pendiente * circles[i][2] + dominio;// se calcula la distancia en base a los valores predefinidos
			p.diametro = circles[i][2];


			if (args->data.tLastPos.size() != 0)
			{
				float d = distancia2(predecida, p);
				if (d < dist || dist == -1)
				{
					dist = d;
					pos = p;
				}
			}
			else
				pos = p;

			/*
			p_fltXYRadius = (float*) cvGetSeqElem(p_seqCircles, i);
			printf("ball position (x,y) = (%f,%f), radius = %f \n", p_fltXYRadius[0], p_fltXYRadius[1], p_fltXYRadius[2]);

			cvCircle(grayImage, cvPoint(p_fltXYRadius[0], p_fltXYRadius[1]), p_fltXYRadius[2],
								CV_RGB(255, 255, 255), 3, 8, 0);
								*/
		//	cvShowImage("redDetection", segmentated)
		}
		if (args->data.tLastPos.size() == 0 && pos.x != -1)
		{
			args->data.tPos = pos;
			args->data.tLastPos.push_back(pos);
		//	args->data.tLastPos = args->data.tPos;
		}
		else if (dist < 300 && dist != -1)
		{
			//args->data.tLastPos = args->data.tPos;
			args->data.tLastPos.push_back(pos);
			args->data.tPos = pos;
			if (args->data.tLastPos.size() == FRAME2)
				args->data.tLastPos.erase(args->data.tLastPos.begin());
		}
		else
		{
			args->data.tPos = pos;//calcular(args->data.tLastPos);
			//pos = args->data.tPos;
			//args->data.tPos.x += args->data.tPos.x - args->data.tLastPos.x;
			//args->data.tPos.y += args->data.tPos.y - args->data.tLastPos.y;
			//args->data.tLastPos = pos;
		}
	//	printf("Distancia: %.f Altura: %d cm Diametro: %d \n", dist, args->data.tPos.z, args->data.tPos.diametro);
		circle(grayImage, cvPoint(args->data.tPos.x, args->data.tPos.y), args->data.tPos.diametro,
					CV_RGB(255, 0, 255), 2, 8, 0);




		line(grayImage,  cvPoint(imgSize.width / 2, imgSize.height / 2 -5), cvPoint(imgSize.width / 2, imgSize.height / 2 + 5),
				CV_RGB(255,255,255),1, 8, 0);
		line(grayImage,  cvPoint(imgSize.width / 2 -5, imgSize.height / 2), cvPoint(imgSize.width / 2 + 5, imgSize.height / 2),
						CV_RGB(255,255,255),1, 8, 0);



		string line1 = format("SET--> Pitch: %f, Roll: %f, Yaw: %f, Thrust: %d"
				, args->data.copterSets.pitch, args->data.copterSets.roll, args->data.copterSets.yaw, args->data.copterSets.thust);

		string line2 = format("GET--> Pitch: %f, Roll: %f, Yaw: %f, Thrust: %d",
				args->data.copterValues.pitch, args->data.copterValues.roll, args->data.copterValues.yaw, args->data.copterValues.thust);

		string line3 = format("POSICION--> X: %d, Y: %d, Z: %d",
						args->data.tPos.x, args->data.tPos.y, args->data.tPos.z);

		sem_post(&args->mutex);

		putText(grayImage, line1, cvPoint(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 255));
		putText(grayImage, line2, cvPoint(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 255));
		putText(grayImage, line3, cvPoint(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 255));
		imshow("redDetection", grayImage);
		outputVideo.write(grayImage);
/*
		gettimeofday(&end, NULL);

		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;


		printf("Elapsed time: %ld milliseconds\n", mtime);
*/
	//	cvShowImage("redDetection", grayImage);


					//(gris, circulos, CV_HOUGH_GRADIENT, 1, gris.rows / 4, 100, 100,
			//		0, 0);

//		cvShowImage("redMorphology", segmentated);
/*

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
				CV_BLOB_RENDER_COLOR | CV_BLOB_RENDER_CENTROID);
			//	CV_BLOB_RENDER_BOUNDING_BOX | CV_BLOB_RENDER_CENTROID);
		// actualizo los tracks
		cvUpdateTracks(blobs, tracks, 200., 5);
		// dibujo los tracks
		cvRenderTracks(tracks, frame, frame,
				CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);

		cv::Mat frame2(frame);

		imshow("red_ball", frame2);

		// paso frame a un Mat
	//	cv::Mat frame2(frame);

		IplImage *ballImage = cvCreateImage(cvGetSize(frame), img->depth, img->nChannels);
		cvSet(ballImage, CV_RGB(255,255,255));


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
				args->data.tPos.z = pendiente * diametro + dominio;// se calcula la distancia en base a los valores predefinidos
				args->data.tPos.z = diametro;
				sem_post(&args->mutex);
			}

			// dibujo un circulo sobre lo que estoy trackeando
			circle(frame2,
					cvPoint(itTracks->second->centroid.x,
							itTracks->second->centroid.y), radio,
					cvScalar(255, 255, 0), 3, 8, 0);

			cvCircle(ballImage, cvPoint(itTracks->second->centroid.x, itTracks->second->centroid.y), radio,
					CV_RGB(255, 0, 0), 3, 8, 0);

		}

	//	cvShowImage("ballPosition", ballImage);



		// libero la memoria usada para los label y la imagen 0/1
		cvReleaseImage(&labelImg);
*/		cvReleaseImage(&segmentated);


		char k = cvWaitKey(bContinue) & 0xff;

		sem_wait(&args->mutex);
		args->data.key = k;
		sem_post(&args->mutex);

		switch (k) {
		case 27:
		case 'q':
		case 'Q':
			quit = true;
			break;
		case 's':
		case 'S':
			bContinue = false;
			break;
		case 'n':
		case 'N':
			bContinue = true;
			break;
		}

		//libero los blobs
	//	cvReleaseBlobs(blobs);



	}

	cvReleaseStructuringElement(&morphKernel);
	cvReleaseImage(&frame);
	outputVideo.release();

	cvDestroyWindow("red_ball");
	cvDestroyWindow("isCircle");

	return 0;
}

bool isSimilar2(CvScalar color)
{
	int v = 25;
	for (int i = 0; i < muestras2.size(); i++)
	{
		if (color.val[0] > muestras2[i].val[0] - v && color.val[0] < muestras2[i].val[0] + v &&
				color.val[1] > muestras2[i].val[1] - v && color.val[1] < muestras2[i].val[1] + v &&
				color.val[2] > muestras2[i].val[2] - v && color.val[2] < muestras2[i].val[2] + v)
			return true;

	}
	return false;
}

bool isSimilarHsv2(CvScalar color)
{

	if ((color.val[0] > hsvMin2.val[0] && color.val[0] < hsvMax2.val[0] &&
			color.val[1] > hsvMin2.val[1] && color.val[1] < hsvMax2.val[1] &&
			color.val[2] > hsvMin2.val[2] && color.val[2] < hsvMax2.val[2]) ||
			(color.val[0] > hsvMin2.val[0] && color.val[0] < hsvMax2.val[0] &&
						color.val[1] > hsvMin2.val[1] + 100 && color.val[1] < hsvMax2.val[1] &&
						color.val[2] > hsvMin2.val[2] - 40 && color.val[2] < hsvMax2.val[2]))
		return true;


	return false;
}


float distancia2(Posicion pos1, Posicion pos2)
{
	return sqrt(pow(fabs(pos1.x - pos2.x),2) + pow(fabs(pos1.y - pos2.y), 2));
}
Posicion calcular2(std::vector<Posicion> tLastPos)
{
	Posicion centro;
	Posicion desplazamiento;
	desplazamiento.x = 0;
	desplazamiento.y = 0;
	desplazamiento.diametro = 0;
	int e = 0;
	int size = tLastPos.size();
	for (int n = 0; n < (int) tLastPos.size() - 1; n++) {

		//	centro = predecida->getCentro();
			desplazamiento.x += (tLastPos[n].x - tLastPos[n + 1].x) * (n + 1);
			desplazamiento.y += (tLastPos[n].y - tLastPos[n + 1].y) * (n + 1);
			desplazamiento.diametro += (tLastPos[n].diametro - tLastPos[n + 1].diametro) * (n + 1);

		/*	centro.x += tLastPos[n].x;
			centro.y += tLastPos[n].y;
			centro.diametro += tLastPos[n].diametro;
*/
		//	predecida->setCentro(centro);
		//	predecida->getCentro().x =+ cercano.getCentro().x - anterior.getCentro().x;
		//	predecida->getCentro().y =+ cercano.getCentro().y - anterior.getCentro().y;


/*

			centro = promedio->getCentro();
			centro.x += cercano->getCentro().x;
			centro.y += cercano->getCentro().y;
			promedio->setCentro(centro);
			promedio->setRadio(promedio->getRadio() + cercano->getRadio());

			anterior2 = anterior;
			anterior = *cercano;
*/
			e += n + 1;


	}
	if (tLastPos.size() > 1) {

		desplazamiento.x /= e;
		desplazamiento.y /= e;
		desplazamiento.diametro /= e;

		Posicion p;
		p.x = tLastPos[tLastPos.size() - 1].x + desplazamiento.x;
		p.y = tLastPos[tLastPos.size() - 1].y + desplazamiento.y;
		p.diametro = tLastPos[tLastPos.size() - 1].diametro + desplazamiento.diametro;
		return p;
/*
		centro.x /= e;
		centro.y /= e;
		centro.diametro /= e;

		Posicion p;
		p.x = (tLastPos[tLastPos.size() - 1].x + centro.x) / 2;
		p.y = (tLastPos[tLastPos.size() - 1].y + centro.y) / 2;
		p.diametro = (tLastPos[tLastPos.size() - 1].diametro + centro.diametro) / 2;
		return p;*/
	}
	else if (tLastPos.size() == 1)
		return tLastPos[0];
	else
		return desplazamiento;
}
