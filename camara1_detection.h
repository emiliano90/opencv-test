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

void *startRedObjectTracking(void *arg);
//int startRedObjectTracking();
float distancia(Posicion pos1, Posicion pos2);
bool isSimilar(CvScalar color);
bool isSimilarHsv(CvScalar color);
Posicion calcular(std::vector<Posicion> tLastPos);
#endif /* RED_OBJECT_TRACKING_H_ */
