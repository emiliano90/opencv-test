/*
 * red_object_tracking.h
 *
 *  Created on: 01/04/2014
 *      Author: toshiba
 */

#include <opencv/cv.h>
#include "estructuras.h"
#include <string>

#ifndef ALTURA_DETECTION_H_
#define ALTURA_DETECTION_H_
const int FRAME2 = 7;

void *startAlturaDetection(void *arg);
//int startRedObjectTracking();
float distancia2(Posicion pos1, Posicion pos2);
bool isSimilar2(CvScalar color);
bool isSimilarHsv2(CvScalar color);
Posicion calcular2(std::vector<Posicion> tLastPos);
#endif /* ALTURA_DETECTION_H_ */
