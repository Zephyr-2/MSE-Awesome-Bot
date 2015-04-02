#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Arduino.h"
#define NUM_TESTS  8

class IRSensor
{
public:
	IRSensor(int pin)
	{
		(*this).pin = pin;
		pinMode(pin, INPUT);
	}

	float read()
	{
		float voltages[NUM_TESTS];

		for (int i = 0; i < NUM_TESTS; i++)
			voltages[i] = analogRead(pin);

		voltages[0] = getAverage(voltages, NUM_TESTS);
        //return voltages[0];	
		return pow(5913 / voltages[0], 1.127) - 0.037 * voltages[0] + 9.5;
	}
	
private:
	int pin;

	float getAverage(float array[], int arraySize)
	{
		int max = 0, min = 0;
		float sum = array[0];

		for (int i = 1; i < arraySize; i++)
		{
			sum += array[i];

			if (array[i] > array[max])
				max = i;

			if (array[i] < array[min])
				min = i;
		}

		sum -= array[min] + array[max];

		return sum / (arraySize - 2);
	}
};

#endif
