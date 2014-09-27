/*
 * red_object_tracking.h
 *
 *  Created on: 01/04/2014
 *      Author: toshiba
 */

#include <opencv/cv.h>
#include "estructuras.h"
#include <string>

#ifndef RED_OBJECT_TRACKING_H_
#define RED_OBJECT_TRACKING_H_
const int FRAME = 7;

void *startCamara1(void *arg);
//int startRedObjectTracking();
float distancia(Posicion pos1, Posicion pos2);
bool isSimilar(CvScalar color);
bool isSimilarHsv(CvScalar color);
Posicion calcular(std::vector<Posicion> tLastPos);
void CallBackClick(int event, int x, int y, int flags, void* userdata);
CvPoint buscarRobot(CvArr *source, std::vector<Posicion> tLastPos);
void sumarScalar(CvScalar &val, CvScalar val2);
void dividirScalar(CvScalar &val, float val2);
#endif /* RED_OBJECT_TRACKING_H_ */
