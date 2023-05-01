#pragma once
#include "Constants.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class RCRPID
{
public:
	RCRPID(volatile float*, float*, volatile float*, float, float, float, float, int, int);//constructor
	void Compute();
private:
	float kp;                  // * (P)roportional Tuning Parameter
	float ki;                  // * (I)ntegral Tuning Parameter
	float kd;                  // * (D)erivative Tuning Parameter
	float n;

	volatile float* myInput;
	float* myOutput;
	volatile float* mySetpoint;

	float ITerm, lastError, lastDTerm = 0;
	unsigned long lastTime;
	int outMin, outMax;

};
