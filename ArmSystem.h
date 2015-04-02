#ifndef ARMSYSTEM_H
#define ARMSYSTEM_H

#define MOTOR_BRAKE  1500

#include "Arduino.h"
#include "PID.h"
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <usTimer2.h>

class ArmSystem
{
  private:
  
  int targetelevation;
  int targetlength;
  int targetclawopen;
  
  float elevatorpositionleft;
  float elevatorpositionright;
  float armposition;
  float clawposition;

  public:
  Servo left_elevator;
  Servo right_elevator;
  Servo arm;
  Servo claw;
  
  I2CEncoder encoder_left_elevator;
  I2CEncoder encoider_right_epevator;
  I2CEncoder encoder_arm;
  I2CEncoder encoder_claw;
  
  PID pid_elevator;
  
  ArmSystem();
  void moveelevator(int targetelevation);
  void extendarm(int targetlength);
  void openclaw(int targetclawopen);
};

#endif
