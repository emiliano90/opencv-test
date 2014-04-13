/*
 * estructuras.h
 *
 *  Created on: 08/04/2014
 *      Author: toshiba
 */

#ifndef ESTRUCTURAS_H_
#define ESTRUCTURAS_H_

//DEFINIR ESTOS VALORES PARA EL CALCULO DE LA DISTANCIA
const int DIST_CERCA = 50;//cm
const int DIST_LEJOS = 300;//cm

const int DIAM_CERCA = 150;//px
const int DIAM_LEJOS = 30;//px

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

struct Datos {
	Posicion tPos;
	Size imgSize;
	bool bActual;
};

struct ThreadAttr {
	sem_t mutex;
	Datos data;

};


#endif /* ESTRUCTURAS_H_ */
