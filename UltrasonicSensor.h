#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Arduino.h"

#define NUM_TESTS  3
#define NUM_AVERAGE   6

class UltrasonicSensor
{
public:
	UltrasonicSensor(int pingPin, int dataPin)
	{
		this->pingPin = pingPin;
		this->dataPin = dataPin;

		pinMode(pingPin, OUTPUT);
		pinMode(dataPin, INPUT);

                counter = 0;
	}

	float read()
	{
                float value = 0;
                
                for(int i = 0; i < 2; i++) {
		  float values[5];

		  for (int i = 0; i < 4; i++) {
                    digitalWrite(pingPin, HIGH);
		    delayMicroseconds(10);
		    digitalWrite(pingPin, LOW);
                    values[i] =  pulseIn(dataPin, HIGH, 10000);
                  }

		  value += getAverage(values, 4);
                }
                
                if(++counter == 7) counter = 0;
                lastValues[counter] = (lastValues[0] + lastValues[1] + lastValues[2] + lastValues[3] + lastValues[4] + lastValues[5] + lastValues[6] + value) / 9;
                
                return lastValues[counter] / 56.18 - 1.66;
	}

private:
	int pingPin;
	int dataPin;
        float lastValues[7];
        int counter;

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
