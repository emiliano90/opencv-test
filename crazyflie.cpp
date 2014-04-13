//============================================================================
// Name        : testDiego.cpp
// Author      : Diego Avila
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <semaphore.h>
#include "estructuras.h"
#include "crazyflie.h"

using namespace std;

void *startCrazyFlie(void *arg) {
	ThreadAttr *args = (ThreadAttr *) arg;

	cout << "!!!Hello sWorld!!!" << endl;

	CCrazyRadio *crRadio = new CCrazyRadio("radio://0/10/250K");

	if (crRadio->startRadio()) {
		CCrazyflie *cflieCopter = new CCrazyflie(crRadio);

		cout << "Connected to the copter" << endl;

		cout << "Setting thrust to 10001" << endl;
		cflieCopter->setThrust(0);

		Posicion ballPos = getPosicion(args, cflieCopter);

		// Enable sending the setpoints. This can be used to temporarily
		// stop updating the internal controller setpoints and instead
		// sending dummy packets (to keep the connection alive).
		cflieCopter->setSendSetpoints(true);

		while (cflieCopter->cycle()) {

			cflieCopter->setThrust(40000);
			cflieCopter->setPitch(-1);

			printf(
					"Ball antes ==> {x: %d , y: %d, z: %d diametro: %d Roll: %f Yaw: %f Pitch: %f  \r\n",
					args->data.tPos.x, args->data.tPos.y, args->data.tPos.z, args->data.tPos.diametro, cflieCopter->roll(),
					cflieCopter->yaw(), cflieCopter->pitch());

			ballPos = getPosicion(args, cflieCopter);

			printf(
					"Ball Corregida ==> {x: %d , y: %d } z: %d Roll: %f Yaw: %f Pitch: %f  \r\n",
					ballPos.x, ballPos.y, ballPos.z, cflieCopter->roll(),
					cflieCopter->yaw(), cflieCopter->pitch());

		}

		cout << endl << "Aterrizando..." << endl;
		int thrust = cflieCopter->thrust();
		while (cflieCopter->cycle() && thrust > 0) {
			thrust--;
			cflieCopter->setThrust(thrust);

			cout << cflieCopter->thrust() << endl;

			sleep(0.5);
		}

		cflieCopter->setThrust(0);

		delete cflieCopter;
	} else {
		cerr << "Could not connect to dongle. Did you plug it in?" << endl;
	}

	cout << "deleting radio object" << endl;
	delete crRadio;

	return 0;
}

Posicion getPosicion(void *threadAttr, CCrazyflie *copter) {
	Posicion ballPos;
	ThreadAttr *args = (ThreadAttr *) threadAttr;

	sem_wait(&args->mutex);
	Point centro;
	centro.y = args->data.imgSize.height / 2;
	centro.x = args->data.imgSize.width / 2;

	//empezamos trabajando con el roll
	float ady = args->data.tPos.x - centro.x;
	float op = centro.y - args->data.tPos.y;

	float ang1 = atan(op / ady);
	if (ady < 0 && op < 0)
		ang1 += M_PI;
	else if (ady < 0)
		ang1 += M_PI;

	float hipo = op / sin(ang1);

	//si el roll es menor a 0 hay que restarselo a ang1
	//si el roll es mayor a 0 hay que sumarselo a ang1

	ang1 += gradosARad(copter->roll());

	float op2 = sin(ang1) * hipo;
	ballPos.x = cos(ang1) * hipo;

	//seguimos trabajando con el pitch(cabeceo)
//	float dist = centro.x - args->data.tPos.x;
//	float op = centro.y - args->data.tPos.y;

	float ang2 = asin(op2 / args->data.tPos.z);
	/*	if(args->data.tPos.z < 0 && op2 < 0)
	 ang1 =+ M_PI;
	 else if (args->data.tPos.z < 0)
	 ang1 =+ M_PI;
	 */
	ang2 += gradosARad(copter->pitch());

	ballPos.y = sin(ang2) * args->data.tPos.z;
	ballPos.z = cos(ang2) * args->data.tPos.z;

	sem_post(&args->mutex);

	return ballPos;
}
float radAGrados(float rad) {
	return rad * 180 / M_PI;
}
float gradosARad(float grados) {
	return grados * M_PI / 180;
}
