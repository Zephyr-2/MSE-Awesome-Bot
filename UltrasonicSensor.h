/**
 * Contains data for one ultrasonic sensor. Uses code from the
 * Arduino tutorial found here:
 * http://arduino.cc/en/Tutorial/Ping?from=Tutorial.UltrasoundSensor
 * 
 * Author: Robert Meagher
 * Written for MSE2202B 2015
 */

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Arduino.h"

class UltrasonicSensor
{
public:
	/**
	 * Initializes the ultrasonic sensor
	 *
	 * @param	pingPin	The pin that sends data to the sensor
	 * @param	echoPin	The pin that sends data from the sensor
	 */
	UltrasonicSensor(int pingPin, int echoPin)
	{
		this->pingPin = pingPin;
		this->echoPin = echoPin;

		slo = 0;
		t_slo = millis();
		read();
	}

	/**
	 * Reads data from the ultrasonic sensor, and converts
	 * it to centimetres
	 */
	void read()
	{
		las = cur;

		// Setting the pin mode every reading allows the ultrasonic's
		// ping and data pins to be the same
		pinMode(pingPin, OUTPUT);
		digitalWrite(pingPin, LOW);
		delayMicroseconds(5);
		digitalWrite(pingPin, HIGH);
		delayMicroseconds(5);
		digitalWrite(pingPin, LOW);

		pinMode(echoPin, INPUT);
		cur = pulseIn(echoPin, HIGH) / 58;

		slo = 1000.0 * (cur - las) / (millis() - t_slo);
		t_slo = millis();
	}

	/**
	 * Returns the current ultrasonic reading
	 *
	 * @return 	The current ultrasonic reading
	 */
	long current()
	{
		return cur;
	}

	/**
	 * Returns the last ultrasonic reading
	 *
	 * @return 	The last ultrasonic reading
	 */
	long last()
	{
		return las;
	}

	/**
	 * Returns the slope of the ultrasonic readings
	 *
	 * @return 	The slope of the ultrasonic readings
	 */
	float slope()
	{
		return slo;
	}

private:
	int pingPin;			// The pin that sends data to the sensor
	int echoPin;			// The pin that sends data from the sensor
	long cur;				// The current ultrasonic reading
	long las;				// The last ultrasonic reading
	long t_slo;				// The time of the last reading
	float slo;				// The slope of the ultrasonic readings
};

#endif
