#ifndef DRIVESYSTEM_H
#define DRIVESYSTEM_H

#define DRIVE  0
#define TURN   1
#define STOP   2

#define MOTOR_MAX    2100
#define MOTOR_MIN    800
#define MOTOR_BRAKE  1500

#define CAL_WIDTH     10

#include "Arduino.h"
#include "PID.h"
#include "Point.h"
#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

class DriveSystem
{
public:
  DriveSystem();

  int lastTime;
  void drive(int targetSpeed);
  void brake();
  void turn(int targetSpeed, float targetTheta);

  float getTheta()    { 
    return currentTheta;    
  }
  Point getPosition() { 
    return currentPosition; 
  }
  void update();

  Servo motor_left;
  Servo motor_right;
  I2CEncoder encoder_left;
  I2CEncoder encoder_right;

private:
  void updatePositionAndTheta();
  float constrainTheta(float theta);

  PID pid_drive;
  PID pid_turn_angle;

  float encoder_left_last;    //last reading from encoder 1
  float encoder_right_last;   //last reading from encoder 2

  Point currentPosition;      //current position from origin
  float currentTheta;

  int targetSpeed;            //target speed as a percent of max speed
  float targetTheta;

  byte state;
};

#endif

