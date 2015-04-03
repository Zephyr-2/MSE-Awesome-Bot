#ifndef ARMSYSTEM_H
#define ARMSYSTEM_H

#include "Arduino.h"
#include "PID.h"
#include <Servo.h>
#include <I2CEncoder.h>
#include <Wire.h>

#define MOTOR_BRAKE			1500
#define ELEVATOR_THRESHOLD	0.1
#define ARM_THRESHOLD		0.1
#define CLAW_THRESHOLD		0.1
#define THRESHOLD               1000

#define ARM_MAX
#define CLAW_MAX

class ArmSystem
{
public:
	ArmSystem();
	
	void setElevator(int targetElevation);
	void setArm(int targetlength);
	void setClaw(int targetclawopen);
	
	bool elevatorAtPosition();
	bool armAtPosition();
	bool clawAtPosition();

	void update();
	
	Servo motor_left;
	Servo motor_right;
	Servo arm;
	Servo claw;

	I2CEncoder encoder_left;
	I2CEncoder encoder_right;
	I2CEncoder encoder_arm;
	I2CEncoder encoder_claw;

	PID pid_elevator;

private:
	int targetElevator;
	int targetArm;
	int targetClaw;
};

#endif
