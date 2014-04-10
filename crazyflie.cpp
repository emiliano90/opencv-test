//============================================================================
// Name        : testDiego.cpp
// Author      : Diego Avila
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <CCrazyflie.h>
#include <semaphore.h>
#include "estructuras.h"

using namespace std;

void *startCrazyFlie(void *arg)
{
	ThreadAttr *args = ( ThreadAttr *)arg;

	cout << "!!!Hello sWorld!!!" << endl;

	CCrazyRadio *crRadio = new CCrazyRadio("radio://0/10/250K");

	if (crRadio->startRadio()) {
		CCrazyflie *cflieCopter = new CCrazyflie(crRadio);

		cout << "Connected to the copter" << endl;

		cout << "Setting thrust to 10001" << endl;
		cflieCopter->setThrust(10001);

		Posicion ballPos = getPosicion(arg, cflieCopter);

		// Enable sending the setpoints. This can be used to temporarily
		// stop updating the internal controller setpoints and instead
		// sending dummy packets (to keep the connection alive).
		cflieCopter->setSendSetpoints(true);

		int i = 0;
		float pitch = 0;
		float roll = 0;

		while (cflieCopter->cycle() && i < 500) {

			cout << "main loop " << i << endl;
			// Main loop. Currently empty.

			/* Examples to set thrust, and RPY: */

			// Range: 10001 - (approx.) 60000
			cflieCopter->setThrust(41000);

			pitch = cflieCopter->pitch();
			roll = cflieCopter->roll();

			cflieCopter->setPitch(pitch * -1);
			cflieCopter->setRoll(roll * -1);

			cout << "----------------------" << "\t FLIGHT INFO "
					<< "----------------------" << endl << "  Pitch: "
					<< cflieCopter->pitch() << "  Roll: " << cflieCopter->roll()
					<< "  Yaw: " << cflieCopter->yaw() << endl
					<< "--------------------------------------------" << endl;
			i++;
		}

		cout << endl << "Aterrizando..." << endl;
		int thrust = cflieCopter->thrust();
		while (cflieCopter->cycle() && thrust > 0) {
			thrust--;
			cflieCopter->setThrust(thrust);

			cout << cflieCopter->thrust() << endl;

			sleep(0.5);
		}

		cout << "main loop end " << i << endl
				<< "setting thrust to 0. motors down" << endl;
		cflieCopter->setThrust(0);

		delete cflieCopter;
	} else {
		cerr << "Could not connect to dongle. Did you plug it in?" << endl;
	}

	cout << "deleting radio object" << endl;
	delete crRadio;

	return 0;
}

Posicion getPosicion(void *threadAttr, CCrazyflie *copter)
{
	Posicion ballPos;
	ThreadAttr *args = ( ThreadAttr *)threadAttr;

	sem_wait(&args->mutex);
	Point centro;
	centro.y = args->data.imgSize.height / 2;
	centro.x = args->data.imgSize.width / 2;

	//empezamos trabajando con el roll
	float ady = centro.x - args->data.tPos.x;
	float op = centro.y - args->data.tPos.y;

	float ang1 = atan(op/ady);
	if(ady < 0 && op < 0)
		ang1 =+ M_PI;
	else if (ady < 0)
		ang1 =+ M_PI;

	float hipo = op / sin(ang1);

	//si el roll es menor a 0 hay que restarselo a ang1
	//si el roll es mayor a 0 hay que sumarselo a ang1

	ang1 =+ gradosARad(copter->roll());

	float op2 = sin(ang1) * hipo;
	ballPos.x = cos(ang1) * hipo;

	//seguimos trabajando con el pitch(cabeceo)
//	float dist = centro.x - args->data.tPos.x;
//	float op = centro.y - args->data.tPos.y;

	float ang1 = asin(op2 / args->data.tPos.z);
/*	if(args->data.tPos.z < 0 && op2 < 0)
		ang1 =+ M_PI;
	else if (args->data.tPos.z < 0)
		ang1 =+ M_PI;
*/
	ang1 =+ gradosARad(copter->pitch());

	ballPos.y = sin(ang1) * args->data.tPos.z;
	ballPos.z = cos(ang1) * args->data.tPos.z;

	sem_post(&args->mutex);

	return ballPos;
}
float radAGrados(float rad)
{
	return rad * 180 / M_PI;
}
float gradosARad(float grados)
{
	return grados * M_PI / 180;
}
