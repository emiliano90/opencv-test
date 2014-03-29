/*
 * Circle.h
 *
 *  Created on: 26/03/2014
 *      Author: toshiba
 */

#ifndef CIRCLE_H_
#define CIRCLE_H_
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


class Circle {
private:
	CvPoint centro;
	int radio;


public:

	Circle();
	Circle(CvPoint center, int rad){
		centro = center;
		radio = rad;
	}

	virtual ~Circle();

	CvPoint getCentro()
	{
	    return centro;
	}

	void setCentro(CvPoint centro)
	{
	    this->centro = centro;
	}

	int getRadio()
	{
	    return radio;
	}

	void setRadio(int radio)
	{
	    this->radio = radio;
	}
};

#endif /* CIRCLE_H_ */
