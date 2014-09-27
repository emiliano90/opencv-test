/*
 * crazyflie.h
 *
 *  Created on: 08/04/2014
 *      Author: toshiba
 */

#ifndef CRAZYFLIE_H_
#define CRAZYFLIE_H_

#include <CCrazyflie.h>
#include "PIDRP.h"
#include "PID.h"

//Roll/pitch limitXZ
const float CAP = 12;
//Thrust limit
const int TH_CAP = 55000;

 void *startCrazyFlie(void *arg);
//int startCrazyFlie();
 Posicion getPosicion(void *threadAttr, CCrazyflie *copter);
 float radAGrados(float rad);

 float gradosARad(float grados);

#endif /* CRAZYFLIE_H_ */
