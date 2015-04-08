/**
 * A PID class that uses a known error value over time to adjust
 * outputs. Must have a known calibration setting prior to use
 *
 * For more information, see:
 * http://en.wikipedia.org/wiki/PID_controller
 * 
 * Unused in the current code implementation, but is ready to
 * use in future iterations
 *
 * Author: Robert Meagher
 * Written for MSE2202B 2015
 */ 

#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
public:
	/**
	 * Creates the PID controller and initializes it with
	 * predefined constants
	 * 
	 * @param Kp	The proportional constant
	 * @param Ki	The integral constant
	 * @param Kd	The differential constant
	 */
	PID(float Kp, float Ki, float Kd)
	{
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		
		reset();
	}
	
	/**
	 * Updates the PID with the most recent error value
	 *
	 * @param error 	The current error value of the PID
	 * @return	The amount to adjust the system input to minimize error
	 */
	float update(float error)
	{
		area += error * (millis() - lastTime) / 1000.0;
		float u = Kp * error + Ki * area + Kd * (error - lastError) / (millis() - lastTime) * 1000;
		lastError = error;
		lastTime = millis();
		return u;
	}
	
	/**
	 * Resets the PID, and clears any stored error values and
	 * integral constants
	 */
	void reset()
	{
		lastError = 0;
		lastTime = millis();
		area = 0;
	}
	
private:
	float Kp;			//The propotional constant
	float Ki;			//The integrator constant
	float Kd;			//The differential constant
	float lastError;	//The last error reading
	float lastTime;		//The last time the PID was updated
	float area;			//The current area calculated by the integrator
};

#endif