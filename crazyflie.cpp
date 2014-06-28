//============================================================================
// Name        : testDiego.cpp
// Author      : Diego Avila
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <iostream>

#include <semaphore.h>
#include "estructuras.h"
#include "crazyflie.h"
#include <cv.h>

using namespace std;

void *startCrazyFlie(void *arg)
{
	ThreadAttr *args = ( ThreadAttr *)arg;

	cout << "!!!Hello sWorld!!!" << endl;

	CCrazyRadio *crRadio = new CCrazyRadio("radio://0/10/250K");

	if (crRadio->startRadio()) {
		CCrazyflie *cflieCopter = new CCrazyflie(crRadio);

		cout << "Connected to the copter" << endl;

		PID_RP* r_pid = new PID_RP(0.05, .00025, 0.25, 0, 0, 250, -100, 0, 1);
		PID_RP* p_pid = new PID_RP(0.04, 0.00025, 0.25, 0, 0, 250, -100, 0, 1);//0.03  25
		PID* t_pid = new PID(15, 40, 1000, 0, 0, 250, -100, 0, 1);


		Posicion center;
		center.x = args->data.imgSize.width / 2;
		center.y = args->data.imgSize.height / 2;

		// Enable sending the setpoints. This can be used to temporarily
		// stop updating the internal controller setpoints and instead
		// sending dummy packets (to keep the connection alive).
		cflieCopter->setSendSetpoints(true);


	/*	float pitch = 0; //adelante positivo, atras negativo
		float roll = 0; //derecha negativo, izquierda positivo
		float yaw = 0; //empieza en 0 cuando enciendo el robot
		int thrust = 0; // Range: 10001 - (approx.) 60000
	*/	bool bQuit = false;

		int safety = 7;

		float roll_sp = 0;
		float pitch_sp = 0;
		float thrust_sp = 0;
		bool bAterrizar = false;


		while (cflieCopter->cycle() && !bQuit && safety != 0) {

			sem_wait(&args->mutex);
			if (args->data.tPos.x != -1 && !bAterrizar)
			{
				safety = 10;

				float roll = r_pid->update(args->data.imgSize.width / 2 - args->data.tPos.x);	//roll
				float pitch = p_pid->update(args->data.imgSize.height / 2- args->data.tPos.y);	//pitch
				float thrust = t_pid->update(120 - args->data.tPos.z);	//thust
				float roll_sp = roll;
				float pitch_sp = pitch;
				//float thrust_sp = thrust + 40000;

				if(roll_sp > CAP)
					roll_sp = CAP;
				else if(roll_sp < -CAP)
					roll_sp = -CAP;

				if (pitch_sp > CAP)
					pitch_sp = CAP;
				else if (pitch_sp < -CAP)
					pitch_sp = -CAP;

				if (thrust_sp > TH_CAP)
					thrust_sp = TH_CAP;
				else if (thrust_sp < 0)
					thrust_sp = 0;

				args->data.copterValues.pitch = cflieCopter->pitch();
				args->data.copterValues.roll = cflieCopter->roll();
				args->data.copterValues.yaw = cflieCopter->yaw();
				args->data.copterValues.thust = cflieCopter->thrust();

				args->data.copterSets.pitch = -pitch_sp;
				args->data.copterSets.roll = roll_sp;
				args->data.copterSets.yaw = 0;
				args->data.copterSets.thust = thrust_sp;

				cflieCopter->setPitch(-pitch_sp);
				cflieCopter->setRoll(roll_sp);
				cflieCopter->setThrust(thrust_sp);
				cflieCopter->setYaw(0);



			//	printf("Datos: Pitch: %f  Roll: %f  Yaw: %f Thrust: %d Bateria> %f\n", cflieCopter->pitch(), cflieCopter->roll(), cflieCopter->yaw(), cflieCopter->thrust(), cflieCopter->batteryLevel());
				printf("Enviado: Pitch: %f  Roll: %f  Thrust: %f \n", pitch_sp, roll_sp, thrust_sp);
			//	printf("Posicion: x: %d  y: %d z: %d \n", args->data.tPos.x, args->data.tPos.y, args->data.tPos.z);
			}
			else
				safety --;

			if(bAterrizar)
			{
				cout << endl << "Aterrizando..." << endl;
				cflieCopter->setPitch(0);
				cflieCopter->setRoll(0);

				args->data.copterSets.pitch = 0;
				args->data.copterSets.roll = 0;

			//	int thrust = cflieCopter->thrust();
				while (cflieCopter->cycle() && thrust_sp > 0) {
					thrust_sp -= 5000;
					cflieCopter->setThrust(thrust_sp);
					args->data.copterSets.thust = thrust_sp;

					cout << cflieCopter->thrust() << endl;

					usleep(800000); //0,5 segundo
				}


				cout<< "setting thrust to 0 motors down" << endl;
				cflieCopter->setThrust(0);
			}

			switch (args->data.key) {
			case 27:
			case 'q':
			case 'Q':
				bQuit = true;
				break;
			case 'x':
			case 'X':
				thrust_sp = 0;
				break;
			case 'c':
			case 'C':
				bAterrizar = true;
				break;
			case 'v':
			case 'V':
				bAterrizar = false;
				break;
			case 'a':
			case 'A':
				if (thrust_sp >= 35000)
					thrust_sp += 1250;
				else
					thrust_sp = 35000;
				break;
			case 'z':
			case 'Z':
				thrust_sp -= 1250;
				break;
			case '\270': //8
				pitch_sp = 4;
				break;
			case '\265': //5
				pitch_sp -= 4;
				break;
			case '\264': //4
				roll_sp = 4;
				break;
			case '\266': //64
				roll_sp -= 4;
				break;
			}
			args->data.key = 0;

			sem_post(&args->mutex);

			usleep(35000);//0,035 segundos

		}


		cout << endl << "Aterrizando..." << endl;
		cflieCopter->setPitch(0);
		cflieCopter->setRoll(0);

		args->data.copterSets.pitch = 0;
		args->data.copterSets.roll = 0;

		int thrust = cflieCopter->thrust();
		while (cflieCopter->cycle() && thrust > 0) {
			thrust -= 5000;
			cflieCopter->setThrust(thrust);
			args->data.copterSets.thust = thrust;

			cout << cflieCopter->thrust() << endl;

			usleep(800000); //0,5 segundo
		}


		cout<< "setting thrust to 0 motors down" << endl;
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
		ang1 = M_PI;
	else if (ady < 0)
		ang1 = M_PI;

	float hipo = op / sin(ang1);

	//si el roll es menor a 0 hay que restarselo a ang1
	//si el roll es mayor a 0 hay que sumarselo a ang1

	ang1 = gradosARad(copter->roll());

	float op2 = sin(ang1) * hipo;
	ballPos.x = cos(ang1) * hipo;

	//seguimos trabajando con el pitch(cabeceo)
//	float dist = centro.x - args->data.tPos.x;
//	float op = centro.y - args->data.tPos.y;

	float ang2 = asin(op2 / args->data.tPos.z);

/*	if(args->data.tPos.z < 0 && op2 < 0)
		ang1 = M_PI;
	else if (args->data.tPos.z < 0)
		ang1 = M_PI;
*/

	ang2 = gradosARad(copter->pitch());

	ballPos.y = sin(ang2) * args->data.tPos.z;
	ballPos.z = cos(ang2) * args->data.tPos.z;

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
/*
void control(void *arg, CCrazyflie* cflieCopter)
{
	ThreadAttr *args = ( ThreadAttr *)arg;

// pitch = depth, roll = x, thrust = y
	//pitch = y
	//roll = x
	//thrust = distancia
	int safety = 10
	while (true)
	{

	//if x and y and depth:
		int safety = 10;
		float roll = r_pid.update(args->data.imgSize.width / 2 - args->data.tPos.x);	//roll
		float pitch = p_pid.update(args->data.imgSize.height / 2- args->data.tPos.y);	//pitch
		float thrust = t_pid.update(120- args->data.tPos.z);	//thust
		float roll_sp = -roll;
		float pitch_sp = -pitch;
		float thrust_sp = thrust + 40000;
		if(roll_sp > CAP)
			roll_sp = CAP;
		else if(roll_sp < -CAP)
			roll_sp = -CAP;

		if (pitch_sp > CAP)
			pitch_sp = CAP;
		else if (pitch_sp < -CAP)
			pitch_sp = -CAP;

		if (thrust_sp > TH_CAP)
			thrust_sp = TH_CAP;
		else if (thrust_sp < 0)
			thrust_sp = 0;

		cflieCopter->setPitch(pitch_sp);
		cflieCopter->setRoll(roll_sp);
		cflieCopter->setThrust(thrust_sp);
		cflieCopter->setYaw(0);

	else
		safety--;
	if (safety < 0)
		break;
	}
*/
