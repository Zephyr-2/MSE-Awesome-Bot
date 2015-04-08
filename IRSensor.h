/**
 * Contains data for reading from one infrared sensor.
 * 
 * Author: Robert Meagher
 * Written for MSE2202B 2015
 */

#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Arduino.h"

#define NUM_TESTS  4
#define NUM_RUNS   15.0

class IRSensor
{
public:
	/**
	 * Initializes the infrared sensor
	 *
	 * @param	pin	The pin that sends data to the sensor.
	 *				Must be an analog pin.
	 */
	IRSensor(int pin)
	{
		(*this).pin = pin;
		pinMode(pin, INPUT);

		cur = 0;
	}
	/**
	 * Reads new data from the infrared sensor, and converts
	 * the data to cm. Averages multiple values over time to
	 * reduce noise.
	 */
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

		cur = 159.36 * exp(-0.005*voltage);
	}

	/**
	 * Returns the current infrared reading
	 *
	 * @return 	The current infrared reading
	 */
	float current()
	{
		return cur;
	}

	/**
	 * Returns the last infrared reading
	 *
	 * @return 	The last infrared reading
	 */
	float last()
	{
		return las;
	}
	
private:
	/**
	 * Takes an array as an argument and returns the average
	 * of the values in the array, not counting the highest
	 * and lowest values
	 *
	 * @param	array 		The values to average
	 * @param	arraySize	The number of values
	 * @return 	The average values in the array
	 */
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

	int pin;	// The pin connected to the infrared sensor
	float cur;	// The current infrared reading
	float las;	// The last infrared reading
};

#endif
