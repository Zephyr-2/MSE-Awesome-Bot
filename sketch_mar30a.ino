#include "DriveSystem.h"
#include "IRSensor.h"
#include "PID.h"
#include "Point.h"
#include "UltrasonicSensor.h"
#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

DriveSystem drive;
UltrasonicSensor us(9, 10);
long time;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  drive.motor_left.attach(2);
  drive.motor_right.attach(8);
  
  drive.encoder_right.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
  drive.encoder_left.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  drive.encoder_left.setReversed(true);  // adjust for positive count when moving forward

  time = millis();
}

void loop()
{
  Serial.println(us.read());
  drive.update();
  
  
  delay(10);
}
