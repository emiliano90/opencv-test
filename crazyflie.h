/*
 * crazyflie.h
 *
 *  Created on: 08/04/2014
 *      Author: toshiba
 */

#include <CCrazyflie.h>


#ifndef CRAZYFLIE_H_
#define CRAZYFLIE_H_

float radAGrados(float rad);
float gradosARad(float grados);
Posicion getPosicion(void *threadAttr, CCrazyflie *copter);
void *startCrazyFlie(void *arg);


#endif /* CRAZYFLIE_H_ */
