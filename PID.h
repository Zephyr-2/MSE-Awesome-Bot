#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
public:
	PID(float Kp, float Ki, float Kd)
	{
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		
		reset();
	}
	
	float update(float error)
	{
		area += error * (millis() - lastTime);
		float u = Kp * error + Ki * area + Kd * (error - lastError) / (millis() - lastTime);
		lastError = error;
		lastTime = millis();
		return u;
	}
	
	void reset()
	{
		lastError = 0;
		lastTime = millis();
		area = 0;
	}
	
private:
	float Kp, Ki, Kd;
	float lastError;
	float lastTime;
	float area;
};

#endif
