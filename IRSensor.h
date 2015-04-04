#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Arduino.h"
#define NUM_TESTS  5
#define NUM_RUNS   3.0

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
                float voltage = 0;
                
                for(int i = 0; i < NUM_RUNS; i++) {
		  float voltages[NUM_TESTS];

		  for (int i = 0; i < NUM_TESTS; i++)
			voltages[i] = analogRead(pin);

		  voltage += getAverage(voltages, NUM_TESTS) / NUM_RUNS;
                }
                
                lastValue = 0.15*(pow(5913 / voltage, 1.127) - 0.037 * voltage + 9.5) + 0.85 * lastValue;
		return lastValue;
	}
	
private:
	int pin;
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
