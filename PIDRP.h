/*
 * PIDRP.h
 *
 *  Created on: 04/06/2014
 *      Author: toshiba
 */

#ifndef PIDRP_H_
#define PIDRP_H_

class PID_RP {
	float Kp;
	float Ki;
	float Kd;
	int Derivator;
	int Integrator;
	int Integrator_max;
	int Integrator_min;
	float set_point;
	float power;

	float last_error;
	float last_value;
	float error;

	float P_value;
	float I_value;
	float D_value;
public:
	PID_RP(float P, float I, float D, int Derivator, int Integrator,
	            int Integrator_max, int Integrator_min, float set_point, float power);

	float update(float current_value);
	void setPoint(float set_point);
	virtual ~PID_RP();
};

#endif /* PIDRP_H_ */
