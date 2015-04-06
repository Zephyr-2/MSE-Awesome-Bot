#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Arduino.h"
#define NUM_TESTS  4
#define NUM_RUNS   15.0

class IRSensor
{
public:
	IRSensor(int pin)
	{
		(*this).pin = pin;
		pinMode(pin, INPUT);
                
                slo = 0;
                t_slo = millis();
                read();
	}

	void read()
	{
            las = cur;
            float voltage = 0;
            
            for(int i = 0; i < NUM_RUNS; i++) {
		  float voltages[NUM_TESTS];

		  for (int i = 0; i < NUM_TESTS; i++)
			voltages[i] = analogRead(pin);

		  voltage += getAverage(voltages, NUM_TESTS) / NUM_RUNS;
            }
            
            cur = 0.4*cur + 0.6*(22359*pow(voltage, -0.79) - 12603*pow(voltage, -0.724) - 18.1);
            slo = 1000.0 * (cur - las) / (millis() - t_slo);
            t_slo = millis();
	}

        float current()
        {
          return cur;
        }
        
        float last()
        {
          return las;
        }
        
        float slope()
        {
          return slo;
        }
	
private:
	int pin;
        float cur, las, slo;
        long t_slo;

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
