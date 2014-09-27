// Cristóbal Carnero Liñán <grendel.ccl@gmail.com>


#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cvblob.h>
#include "camara1_detection.h"
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
#include <fstream>
#include <iostream>

using namespace std;

using namespace cvb;
using namespace cv;

ofstream m_myfile;

vector<CvPoint> destinos;
int nDestino;
std::string times;
vector<CvScalar> muestras;
bool bPerdido = false;
CvScalar hsvMax;// = (340, 255, 255);
CvScalar hsvMin;// =(310, 125, 153);
/*muestras[0] = CV_RGB(200, 142, 202);
muestras[1] = CV_RGB(176, 156, 221);
muestras[1] = CV_RGB(182, 72, 151);
muestras[1] = CV_RGB(182, 72, 151);

CvScalar muestra1
CvScalar muestra2 = CV_RGB(176, 156, 221);
CvScalar muestra3 = CV_RGB(182, 72, 151);
CvScalar muestra4 = CV_RGB(182, 72, 151);*/
//rgb(173, 72, 121)
bool isCircle(std::string window, cv::Mat image, int x1, int y1, int ancho,
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
void *startCamara1(void *arg){
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

	hsvMax.val[0] = 179;
	hsvMax.val[1] = 255;
	hsvMax.val[2] = 255;

	hsvMin.val[0] = 135;// 0.75 135
	hsvMin.val[1] = 80;//76 bien  56  0.25 66
	hsvMin.val[2] = 130;//125  //130 0.54 140

	muestras.push_back(CV_RGB(250, 144, 255));

	const int START = 90;
	const int AMPLIAR_Y = 10;
	const int AMPLIAR_X = 3;
	const int CENTRO_Y = START + AMPLIAR_Y * 30 / 2;
	const int HEIGHT_GRAF = AMPLIAR_Y * 30 + 90;

	struct timeval start, end;

	long seconds, useconds, mtime = 0;


	gettimeofday(&start, NULL);


	CvTracks tracks;
/*
//	cvNamedWindow("red_object_tracking", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("camera2", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("redDetection2", CV_WINDOW_AUTOSIZE);
//	cvNamedWindow("isCircle", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("gray2", CV_WINDOW_AUTOSIZE);
*/
//	cvNamedWindow("redMorfhology", CV_WINDOW_AUTOSIZE);

	CvCapture *capture = cvCaptureFromCAM(1);

//	system("v4l2-ctl -d /dev/video1 -i 0 -s 0 --set-fmt-video=width=1280,height=1024,pixelformat=0");

	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);//1280
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);//1024
	double widht = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	double height = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);

	printf("widht: %f, height: %f",widht, height);
//	double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
//	printf("%f",fps);
    //cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 29);

	cvGrabFrame(capture);
	IplImage *img = cvRetrieveFrame(capture);

	CvSize imgSize = cvGetSize(img);
	cvMoveWindow("redDetection2", imgSize.width, 0);
	args->data.imgSize.height = imgSize.height;
	args->data.imgSize.width = imgSize.width;

	IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

	IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5, 5, CV_SHAPE_ELLIPSE, NULL);
	IplConvKernel *se5 = cvCreateStructuringElementEx(5, 5, 2,  2,  CV_SHAPE_ELLIPSE, NULL);

	//Para detectar clicks en la ventana
	cvNamedWindow("Deteccion", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("Deteccion", CallBackClick, NULL);

	//Defino los destinos
	destinos.push_back(cvPoint(320, 240));
	/*
	destinos.push_back(cvPoint(180, 150));
	destinos.push_back(cvPoint(460, 150));
	destinos.push_back(cvPoint(460, 330));
	destinos.push_back(cvPoint(180, 330));
	*/
	nDestino = 0;
	int lastDest = 0;

	//esto es para grabar el video en un archivo
	time_t _tm = time(NULL );

	struct tm * curtime = localtime ( &_tm );

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
	std::string str(buffer);

	times = str;

	string source = "/home/toshiba/tesis/video/";
	source.append(buffer);
	source.append(".avi");

	string source2 = "/home/toshiba/tesis/video/";
	source2.append(buffer);
	source2.append("_2.avi");

	string source3 = "/home/toshiba/tesis/video/";
	source3.append(buffer);
	source3.append("_3.avi");

	string source4 = "/home/toshiba/tesis/video/";
	source4.append(buffer);
	source4.append(".txt");
	m_myfile.open (source4.data(), ios::out | ios::app);

	//"IYUV
	VideoWriter outputVideo;                                        // Open the output
	outputVideo.open(source, CV_FOURCC('X','V','I','D'), 12, cvSize(imgSize.width * 2,imgSize.height), true);

	VideoWriter outputVideo2;                                        // Open the output
	outputVideo2.open(source2, CV_FOURCC('X','V','I','D'), 12, cvSize(imgSize.width,imgSize.height), true);

	VideoWriter outputVideo3;                                        // Open the output
	outputVideo3.open(source3, CV_FOURCC('X','V','I','D'), 12, cvSize(imgSize.width,imgSize.height), true);

	if (!outputVideo.isOpened())
	{
		printf("Could not open the output video for write: %s", source.c_str() );
		return 0;
	}
	if (!outputVideo2.isOpened())
	{
		printf("Could not open the output video2 for write: %s", source2.c_str() );
		return 0;
	}
	//fin grabar video

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
			printf("Red Object Elapsed time: %ld milliseconds, fps: %d \n", mtime, fps);
			fps = 0;
			mtime = 0;
		}

		gettimeofday(&start, NULL);

		IplImage *img = cvQueryFrame(capture);
	//	IplImage *img = cvRetrieveFrame(capture);
	//	cvGrabFrame(capture);

		cvConvertScale(img, frame, 1, 0);

		outputVideo2.write(img);

		cvShowImage("Camara1", frame);

		//cvSize(640,640)
		IplImage *segmentated = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
		IplImage *color2 = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);

		cvCvtColor(frame, frame, COLOR_BGR2HSV);


		// crea una matriz (IplImage* segmented) solo de pixeles blancos (para los rojos) y negros (para el resto)
		for (int j = 0; j < imgSize.height; j++)
			for (int i = 0; i < imgSize.width; i++) {

				CvScalar c = cvGet2D(frame, j, i);
/*
				sumarScalar(c, cvGet2D(frame, j+1, i));
				sumarScalar(c, cvGet2D(frame, j-1, i));
				sumarScalar(c, cvGet2D(frame, j, i+1));
				sumarScalar(c, cvGet2D(frame, j, i-1));
				dividirScalar(c, 5);
*/
				unsigned char f = 255 * !isSimilarHsv(c);
			/*	double b = ((double) c.val[0]) / 255.;
				double g = ((double) c.val[1]) / 255.;
				double r = ((double) c.val[2]) / 255.;
				unsigned char f = 255 * !((r > 0.1 + g) && (r > b));
// && (g - 0.1 < b) && (g > b - 0.5)
  				*/
				cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
				if (!f)
					cvSet2D(color2, j, i, c);
				else
					cvSet2D(color2, j, i, CV_RGB(255, 255, 255));

			}
		cvCvtColor(color2, color2, COLOR_HSV2BGR);
		//cvShowImage("Color", color2);

		// pagina 120 de O'Reilly Learning Opencv
//		cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN,1);


	//	cvMorphologyEx(segmentated, segmentated, NULL, se11, CV_MOP_OPEN, 1); // See completed example for cvClose definition
	//	cvMorphologyEx(segmentated, segmentated, NULL, se5, CV_MOP_CLOSE, 1);  // See completed example for cvOpen  definition


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
	//	Mat grayImage;
		Mat grayImage(imgSize, CV_8UC1, cv::Scalar(0, 0, 0));

		sem_wait(&args->mutex);
		CvPoint p = buscarRobot(segmentated, args->data.tLastPos);




	//	cvtColor(segmented, grayImage, CV_RGB2GRAY);

	//	imshow("HSV-Morphology", segmented);

		//Para hough circles
	//	GaussianBlur( grayImage, grayImage, cv::Size(3, 3), 0, 0 ); // estaba dsps del canny
	//	Canny(grayImage, grayImage, 5, 70, 3);

		//NOOO
	//	GaussianBlur( grayImage, grayImage, cv::Size(15, 15), 0, 0 );
	//	smooth(grayImage, grayImage, CV_GAUSSIAN, 15, 15);

		/*
		vector<cv::Vec3f> circles;
		HoughCircles(grayImage, circles, CV_HOUGH_GRADIENT, 2, imgSize.height/4, 50, 10, 1, 20);//300,20

		Posicion pos;
		pos.x = -1;
		pos.y = -1;
		pos.z = -1;
		float dist = -1;

		sem_wait(&args->mutex);

		Posicion predecida = calcular(args->data.tLastPos);
		for(int i = 0; i < circles.size(); i++){

			Posicion p;
			p.x = circles[i][0];
			p.y = circles[i][1];
			p.z = pendiente * circles[i][2] + dominio;// se calcula la distancia en base a los valores predefinidos
			p.diametro = circles[i][2];

			if (args->data.tLastPos.size() != 0)
			{
				float d = distancia(predecida, p);
				if (d < dist || dist == -1)
				{
					dist = d;
					pos = p;
				}
			}
			else
				pos = p;

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
			if (args->data.tLastPos.size() == FRAME)
				args->data.tLastPos.erase(args->data.tLastPos.begin());
		}
		else
		{
			printf("Robot Lost\n");
			args->data.tPos = pos;//calcular(args->data.tLastPos);
			//pos = args->data.tPos;
			//args->data.tPos.x += args->data.tPos.x - args->data.tLastPos.x;
			//args->data.tPos.y += args->data.tPos.y - args->data.tLastPos.y;
			//args->data.tLastPos = pos;
		}
		*/


		Posicion predecida = calcular(args->data.tLastPos);
		Posicion pos;
		pos.x = p.x;
		pos.y = p.y;
		if (p.x == -1)
			int a = 5;
		float dist = -1;
		if (args->data.tLastPos.size() != 0 && pos.x != -1)
			dist = distancia(predecida, pos);



		if (args->data.tLastPos.size() == 0 && pos.x != -1)
		{
			args->data.tPos = pos;
			args->data.tLastPos.push_back(pos);
		//	args->data.tLastPos = args->data.tPos;
		}
		else if (dist < 100 && dist != -1)
		{
			//args->data.tLastPos = args->data.tPos;
			args->data.tLastPos.push_back(pos);
			args->data.tPos = pos;
			if (args->data.tLastPos.size() == FRAME)
				args->data.tLastPos.erase(args->data.tLastPos.begin());
		}
		else
		{
			printf("Robot Lost\n");
			Posicion p;
			p.x = -1;
			p.y = -1;
			args->data.tPos = p;

			//calcular(args->data.tLastPos);
			//pos = args->data.tPos;
			//args->data.tPos.x += args->data.tPos.x - args->data.tLastPos.x;
			//args->data.tPos.y += args->data.tPos.y - args->data.tLastPos.y;
			//args->data.tLastPos = pos;
		}


	//	printf("Distancia: %.f Altura: %d cm Diametro: %d \n", dist, args->data.tPos.z, args->data.tPos.diametro);
		circle(grayImage, cvPoint(args->data.tPos.x, args->data.tPos.y), 4,
					CV_RGB(255, 0, 255), 2, 8, 0);

		for(int i = 0; i < destinos.size(); i++)
		{

			line(grayImage,  cvPoint(destinos[i].x, destinos[i].y -5), cvPoint(destinos[i].x, destinos[i].y + 5),
					CV_RGB(255,255,255),(nDestino == i) + 1, 8, 0);
			line(grayImage,  cvPoint(destinos[i].x -5, destinos[i].y), cvPoint(destinos[i].x + 5, destinos[i].y),
							CV_RGB(255,255,255), (nDestino == i) + 1, 8, 0);
		}

		args->data.destino.x = destinos[nDestino].x;
		args->data.destino.y = destinos[nDestino].y;

		string line1 = format("SET--> Pitch: %f, Roll: %f, Yaw: %f, Thrust: %d"
				, args->data.copterSets.pitch, args->data.copterSets.roll, args->data.copterSets.yaw, args->data.copterSets.thust);

		string line2 = format("GET--> Pitch: %f, Roll: %f, Yaw: %f, Thrust: %d, Pressure: %f",
				args->data.copterValues.pitch, args->data.copterValues.roll, args->data.copterValues.yaw, args->data.copterValues.thust, args->data.copterValues.pressure);

		string line3 = format("POSICION--> X: %d, Y: %d, Z: %d",
						args->data.tPos.x, args->data.tPos.y, args->data.tPos.z);



		putText(grayImage, line1, cvPoint(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 255));
		putText(grayImage, line2, cvPoint(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 255));
		putText(grayImage, line3, cvPoint(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 255));
		imshow("Deteccion", grayImage);


		//Genero los graficos
		Mat graficos(cvSize(imgSize.width, HEIGHT_GRAF), CV_8UC3, cv::Scalar(255, 255, 255));

		line(graficos,  cvPoint(0, CENTRO_Y), cvPoint(imgSize.width, CENTRO_Y),
								CV_RGB(0,0,0),1, 8, 0);
		line(graficos,  cvPoint(0, CENTRO_Y + 15 * AMPLIAR_Y), cvPoint(imgSize.width, CENTRO_Y + 15 * AMPLIAR_Y),
								CV_RGB(0,0,0),1, 8, 0);
		line(graficos,  cvPoint(0, CENTRO_Y - 15 * AMPLIAR_Y), cvPoint(imgSize.width, CENTRO_Y - 15 * AMPLIAR_Y),
								CV_RGB(0,0,0),1, 8, 0);

		for(int i = 0; i < int(args->data.copterSets.Kp.size() - 1); i++)
		{
			line(graficos,  cvPoint(i * AMPLIAR_X, CENTRO_Y + args->data.copterSets.Kp[i] * AMPLIAR_Y),
					cvPoint((i + 1) * AMPLIAR_X, CENTRO_Y + args->data.copterSets.Kp[i + 1] * AMPLIAR_Y), CV_RGB(255,0,0), 1 ,8, 0);
			line(graficos,  cvPoint(i * AMPLIAR_X, CENTRO_Y + args->data.copterSets.Ki[i] * AMPLIAR_Y),
					cvPoint((i + 1) * AMPLIAR_X, CENTRO_Y + args->data.copterSets.Ki[i + 1] * AMPLIAR_Y), CV_RGB(0,180,0), 1 ,8, 0);
			line(graficos,  cvPoint(i * AMPLIAR_X, CENTRO_Y + args->data.copterSets.Kd[i] * AMPLIAR_Y),
					cvPoint((i + 1) * AMPLIAR_X, CENTRO_Y + args->data.copterSets.Kd[i + 1] * AMPLIAR_Y), CV_RGB(0,0,200), 1 ,8, 0);

			line(graficos,  cvPoint(i * AMPLIAR_X, CENTRO_Y + args->data.copterSets.rolls[i] * AMPLIAR_Y),
					cvPoint((i + 1) * AMPLIAR_X, CENTRO_Y + args->data.copterSets.rolls[i + 1] * AMPLIAR_Y), CV_RGB(255,255,0), 2 ,8, 0);

			line(graficos,  cvPoint(i * AMPLIAR_X, CENTRO_Y + args->data.copterValues.rolls[i] * AMPLIAR_Y),
					cvPoint((i + 1) * AMPLIAR_X, CENTRO_Y + args->data.copterValues.rolls[i + 1] * AMPLIAR_Y), CV_RGB(0,255,255), 2 ,8, 0);

		}
		putText(graficos, "proportional", cvPoint(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 0, 0));
		putText(graficos, "integrate", cvPoint(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 255, 0));
		putText(graficos, "derivate", cvPoint(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 255));
		putText(graficos, "roll", cvPoint(10, 70), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255, 255, 0));

		line(graficos,  cvPoint(imgSize.width - 100, 55), cvPoint(imgSize.width - 10, 55),
					CV_RGB(0,0,0),2, 8, 0);

		line(graficos,  cvPoint(imgSize.width - 55, 10), cvPoint(imgSize.width - 55, 100),
					CV_RGB(0,0,0),2, 8, 0);

		circle(graficos, cvPoint(imgSize.width - 55 - args->data.copterValues.roll * 5, 55), 4,
							CV_RGB(255, 0, 255), -1, 8, 0);

		circle(graficos, cvPoint(imgSize.width - 55, 55 - args->data.copterValues.pitch * 5), 4,
							CV_RGB(255, 0, 0), -1, 8, 0);

		sem_post(&args->mutex);
		imshow("Graficos", graficos);

		Mat color(cvSize(imgSize.width, imgSize.height), CV_8UC3, cv::Scalar(255, 255, 255));
		Mat grande(cvSize(imgSize.width * 2, imgSize.height), CV_8UC3, cv::Scalar(255, 255, 255));
		cvtColor(grayImage, color, CV_GRAY2RGB);
		color.copyTo(grande.colRange(0, imgSize.width).rowRange(0, imgSize.height));
		graficos.copyTo(grande.colRange(imgSize.width, imgSize.width * 2).rowRange(0, HEIGHT_GRAF));
		outputVideo.write(grande);
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
		outputVideo3.write(segmentated);
		cvShowImage("redMorphology", segmentated);
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
	cvReleaseStructuringElement(&se11);
	cvReleaseStructuringElement(&se5);
	cvReleaseImage(&frame);
	outputVideo.release();

	cvDestroyWindow("red_ball");
	cvDestroyWindow("isCircle");

	return 0;
}

bool isSimilar(CvScalar color)
{
	int v = 25;
	for (int i = 0; i < muestras.size(); i++)
	{
		if (color.val[0] > muestras[i].val[0] - v && color.val[0] < muestras[i].val[0] + v &&
				color.val[1] > muestras[i].val[1] - v && color.val[1] < muestras[i].val[1] + v &&
				color.val[2] > muestras[i].val[2] - v && color.val[2] < muestras[i].val[2] + v)
			return true;

	}
	return false;
}

bool isSimilarHsv(CvScalar color)
{

	if ( ((color.val[0] < 9 || (color.val[0] > hsvMin.val[0] && color.val[0] < hsvMax.val[0]) ) &&
			color.val[1] > hsvMin.val[1] && color.val[1] < hsvMax.val[1] &&
			color.val[2] > hsvMin.val[2] && color.val[2] < hsvMax.val[2]) ||
			((color.val[0] < 9 || (color.val[0] > hsvMin.val[0] && color.val[0] < hsvMax.val[0])) &&
						color.val[1] > hsvMin.val[1] + 100 && color.val[1] < hsvMax.val[1] &&
						color.val[2] > hsvMin.val[2] - 40 && color.val[2] < hsvMax.val[2]))
		return true;


	return false;
}


float distancia(Posicion pos1, Posicion pos2)
{
	return sqrt(pow(fabs(pos1.x - pos2.x),2) + pow(fabs(pos1.y - pos2.y), 2));
}
float distancia(CvPoint pos1, CvPoint pos2)
{
	return sqrt(pow(fabs(pos1.x - pos2.x),2) + pow(fabs(pos1.y - pos2.y), 2));
}
Posicion calcular(std::vector<Posicion> tLastPos)
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
		p.x = tLastPos[tLastPos.size() - 1].x - desplazamiento.x;
		p.y = tLastPos[tLastPos.size() - 1].y - desplazamiento.y;
		p.diametro = tLastPos[tLastPos.size() - 1].diametro - desplazamiento.diametro;
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

void CallBackClick(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {

    	 nDestino++;
    	 if (nDestino == destinos.size())
    		 nDestino = 0;
     }
}
void sumarScalar(CvScalar &val, CvScalar val2)
{
	val.val[0] += val2.val[0];
	val.val[1] += val2.val[1];
	val.val[2] += val2.val[2];
//	val.val[3] += val2.val[3];
}
void dividirScalar(CvScalar &val, float val2)
{
	val.val[0] /= val2;
	val.val[1] /= val2;
	val.val[2] /= val2;
//	val.val[3] /= val2;
}
CvPoint buscarRobot(CvArr *source, std::vector<Posicion> tLastPos)
{
	vector<CvPoint> puntos;
	CvPoint p = cvPoint(0, 0);
	CvPoint p2 = cvPoint(0, 0);
	int count = 0;

	//Busco los puntos negros y los guardo en un vector
	for (int j = 0; j < 480; j++)
	{
		for (int i = 0; i < 640; i++) {
			CvScalar c = cvGet2D(source, j, i);//y, x
			if (c.val[0] == 0)
			{
				puntos.push_back(cvPoint(i, j));
			//	p.x += i;
			//	p.y += j;
				count++;
			}
		}

	}
	//armo grupos de puntos de acuerdo a la cercania de los mismos
	vector< vector<CvPoint> > grupos;
	while(puntos.size() != 0)
	{
		vector<CvPoint> row;
		row.push_back(puntos[0]);
		puntos.erase(puntos.begin());
		for (int e = 0; e < puntos.size(); e++)
		{
			if (distancia(puntos[e], row[0]) < 20)
			{
				row.push_back(puntos[e]);
				puntos.erase(puntos.begin() + e);
				e--;
			}
		}
		grupos.push_back( row );
	}

	//Calculo el centro de los grupos
	vector <CvPoint> centros;
	for (int i = 0; i < grupos.size(); i++)
	{
		p.x = 0;
		p.y = 0;
		for (int e = 0 ; e < grupos[i].size(); e++)
		{
			p.x += grupos[i][e].x;
			p.y += grupos[i][e].y;

		}
		if (grupos[i].size() != 0)
		{
			p.x /= grupos[i].size();
			p.y /= grupos[i].size();
			centros.push_back(p);
		}

	}

	//Elijo el grupo mas cercano a destino
	Posicion predecida = calcular(tLastPos);
	CvPoint pred = cvPoint(predecida.x, predecida.y);

	int pos = -1;
	float dist = -1;
	float sizes = -1;
	int pos2 = -1;
	if (tLastPos.size() != 0)
	{
		for (int i = 0; i < centros.size(); i++)
		{
			if (grupos[i].size() > sizes)
			{
				sizes = grupos[i].size();
				pos2 = i;
			}
			if(distancia(centros[i], pred) < dist  || dist == -1)
			{
				dist = distancia(centros[i], pred);
				pos = i;
			}


		}
	}
	else if (tLastPos.size() == 0)
	{

		for (int i = 0; i < grupos.size(); i++)
		{
			if(grupos[pos].size() < grupos[i].size())
				pos = i;

		}
	}

	if (pos != -1 && dist < 80)
	{
		m_myfile << "Predecida X: "<<predecida.x<< " Y: "<< predecida.y<< " Pos X: "<< centros[pos].x<< " Y: "<< centros[pos].y<< " Dist: "<<dist <<" Pixeles: "<<grupos[pos].size()<< endl;//"\n";
		bPerdido = false;
		return centros[pos];
	}
	else if(bPerdido && pos2 != -1)
	{
		m_myfile << "Predecida X: "<<predecida.x<< " Y: "<< predecida.y<< " Pos X: "<< centros[pos2].x<< " Y: "<< centros[pos2].y<< " Dist: "<<dist <<" Pixeles: "<<grupos[pos2].size()<< " Perdidoo "<< endl;//"\n";
		bPerdido = false;
		return centros[pos2];
	}
	else
	{
		m_myfile << "Predecida X: "<<predecida.x<< " Y: "<< predecida.y<< " Dist: "<<dist<< endl;//"\n";
		bPerdido = true;
		return cvPoint(-1,-1);
	}




	/*

	if (count != 0)
	{
		p.x /= count;
		p.y /= count;
	}
	for (int i = 0 ; i < puntos.size(); i++)
		if (distancia(p, puntos[i]) > 200)
			puntos.erase(puntos.begin() + i);
		else
		{
			p2.x += puntos[i].x;
			p2.y += puntos[i].y;

		}
	if (count != 0)
	{
		p2.x /= puntos.size();
		p2.y /= puntos.size();
	}
	p.x = 0;
	p.y = 0;
	for (int i = 0 ; i < puntos.size(); i++)
			if (distancia(p2, puntos[i]) > 70)
				puntos.erase(puntos.begin() + i);
			else
			{
				p.x += puntos[i].x;
				p.y += puntos[i].y;
			}
	if (puntos.size() != 0)
	{
		p.x /= puntos.size();
		p.y /= puntos.size();
	}
	p2.x = 0;
	p2.y = 0;
	for (int i = 0 ; i < puntos.size(); i++)
			if (distancia(p, puntos[i]) > 30)
				puntos.erase(puntos.begin() + i);
			else
			{
				p2.x += puntos[i].x;
				p2.y += puntos[i].y;
			}
	if (puntos.size() != 0)
	{
		p2.x /= puntos.size();
		p2.y /= puntos.size();
	}
	else
	{
		p2.x = -1;
		p2.y = -1;
	}*/
//	return p;
}
