#ifndef ARMSYSTEM_H
#define ARMSYSTEM_H

#include "Arduino.h"
#include "PID.h"
#include <Servo.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>
#include <Wire.h>

#define MOTOR_BRAKE	        1500

#define ARM_MAX 10
#define CLAW_MAX 10

#define ELEVATOR_UNLOAD_BRAKE  1550
#define ELEVATOR_UNLOAD_UP     1750
#define ELEVATOR_UNLOAD_DOWN   1450
#define ELEVATOR_LOAD_BRAKE    1600
#define ELEVATOR_LOAD_UP       1800
#define ELEVATOR_LOAD_DOWN     1550
#define ELEVATOR_THRESHOLD     2

#define ARM_FOREWARD           1800
#define ARM_RETRACT            1200
#define ARM_THRESHOLD          2

#define CLAW_CLOSE             1300
#define CLAW_OPEN              1700
#define CLAW_THRESHOLD         10

#define TOWER_LEFT             1700
#define TOWER_RIGHT            1300
#define TOWER_THRESHOLD        3

class ArmSystem
{
public:
	ArmSystem();
	
	void setElevator(int targetElevation);
	void setArm(int targetlength);
	void setClaw(int targetclawopen);
        void setTower(int targetTheta);
	
	bool elevatorAtPosition();
	bool armAtPosition();
	bool clawAtPosition();
        bool towerAtPosition();

	void update();
	
	Servo motor_left;
	Servo motor_right;
	Servo arm;
	Servo claw;
        Servo tower;

	I2CEncoder encoder_left;
	I2CEncoder encoder_right;
	I2CEncoder encoder_arm;
	I2CEncoder encoder_claw;
        I2CEncoder encoder_tower;

private:
	int targetElevator;
	int targetArm;
	int targetClaw;
        int targetTower;
        
        bool loaded;
};

#endif
