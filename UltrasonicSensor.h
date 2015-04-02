#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Arduino.h"

class UltrasonicSensor
{
public:
	UltrasonicSensor(int pingPin, int dataPin)
	{
		this->pingPin = pingPin;
		this->dataPin = dataPin;

		pinMode(pingPin, OUTPUT);
		pinMode(dataPin, INPUT);

                lastValue = 100;
	}

	float read()
	{
		digitalWrite(pingPin, HIGH);
		delayMicroseconds(10);
		digitalWrite(pingPin, LOW);
		lastValue = 0.7*(0.0174 * pulseIn(dataPin, HIGH, 10000) - 1.56) + 0.3*lastValue;
                return lastValue;
	}

private:
	int pingPin;
	int dataPin;
        float lastValue;
};

#endif
