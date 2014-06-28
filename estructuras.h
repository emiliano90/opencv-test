/*
 * estructuras.h
 *
 *  Created on: 08/04/2014
 *      Author: toshiba
 */

#ifndef ESTRUCTURAS_H_
#define ESTRUCTURAS_H_
#include <semaphore.h>
#include <vector>
//DEFINIR ESTOS VALORES PARA EL CALCULO DE LA DISTANCIA
const int DIST_CERCA = 0;//cm
const int DIST_LEJOS = 110;//cm

const int DIAM_CERCA = 4;//px
const int DIAM_LEJOS = 6;//px

struct Posicion {
	int x;
	int y;
	int z;
	int diametro;

};

struct Size {
	int height;
	int width;
};

struct Point {
	int x;
	int y;
};

struct Copter {
	float roll;
	float pitch;
	float yaw;
	int thust;
};

struct Datos {
	std::vector<Posicion> tLastPos;
	Posicion tPos;
//	Posicion tLastPos;
	Size imgSize;
	bool bActual;
	char key;
	Copter copterSets;
	Copter copterValues;
};

struct ThreadAttr {
	sem_t mutex;
	Datos data;

};


#endif /* ESTRUCTURAS_H_ */
