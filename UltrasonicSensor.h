#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Arduino.h"

#define NUM_TESTS  5

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
                float values[NUM_TESTS];
                
                for(int i = 0; i < NUM_TESTS; i++) {
		  digitalWrite(pingPin, HIGH);
		  delayMicroseconds(10);
		  digitalWrite(pingPin, LOW);
                  values[i] = pulseIn(dataPin, HIGH, 10000);
                  delayMicroseconds(20);
                }
		lastValue = 0.2*(0.0174 * 1.5 * getAverage(values, NUM_TESTS) - 1.56) + 0.8*lastValue;
		return lastValue;
	}

private:
	int pingPin;
	int dataPin;
	float lastValue;

        float getAverage(float array[], int arraySize)
	{
                int max = 0, min = 0;
		float sum = array[0];

		for (int i = 1; i < arraySize; i++) {
                        if(array[i] < array[min])
                          min = i;
                        else if(array[i] > array[max])
                          max = i;
                          
			sum += array[i];
                }

		sum -= array[min] + array[max];

		return sum / (arraySize - 2);
	}
};

#endif
