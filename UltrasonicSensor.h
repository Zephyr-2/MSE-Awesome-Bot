#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Arduino.h"

class UltrasonicSensor
{
public:
	UltrasonicSensor(int pingPin, int echoPin)
	{
		this->pingPin = pingPin;
		this->echoPin = echoPin;

                slo = 0;
                t_slo = millis();
                read();
	}

	void read()
	{
                las = cur;
                
                pinMode(pingPin, OUTPUT);
                digitalWrite(pingPin, LOW);
                delayMicroseconds(2);
                digitalWrite(pingPin, HIGH);
                delayMicroseconds(5);
                digitalWrite(pingPin, LOW);
              
                pinMode(echoPin, INPUT);
                cur = pulseIn(echoPin, HIGH) / 58;
                
                slo = 1000.0 * (cur - las) / (millis() - t_slo);
                t_slo = millis();
	}

        long current()
        {
          return cur;
        }

        long last()
        {
          return las;
        }
        
        float slope()
        {
          return slo;
        }

private:
	int pingPin;
	int echoPin;
        long cur, las, t_slo;
        float slo;
};

#endif
