/**
 * Controls the arm motors of the robot, including
 * the rotating tower, the elevator, the arm motor
 * and the claw motor.
 *
 * Author: Robert Meagher and Jai Sood
 * Written for MSE2202B 2015
 */

#include "ArmSystem.h"

ArmSystem::ArmSystem()
{
	targetElevator = 0;
	targetArm = 0;
	targetClaw = 0;
	targetTower = 0;
	hasBottle = false;
}

void ArmSystem::setElevator(int targetElevator)
{
	this->targetElevator = targetElevator;
}

void ArmSystem::setArm(int targetArm)
{
	this->targetArm = targetArm;
}

void ArmSystem::setClaw(int targetClaw)
{
	this->targetClaw = targetClaw;
}

void ArmSystem::setTower(int targetTower)
{
	this->targetTower = targetTower;
}

bool ArmSystem::elevatorAtPosition()
{
	if(abs(targetElevator - encoder_left.getPosition()) < ELEVATOR_THRESHOLD)
		return true;
	return false;
}

bool ArmSystem::armAtPosition()
{
	if(abs(targetArm - encoder_arm.getPosition()) < ARM_THRESHOLD)
		return true;
	return false;
}

bool ArmSystem::clawAtPosition()
{
	if(abs(targetClaw - encoder_claw.getPosition()) < CLAW_THRESHOLD)
		return true;
	return false;
}

bool ArmSystem::towerAtPosition()
{
	if(abs(targetTower - encoder_tower.getPosition()) < TOWER_THRESHOLD)
		return true;
	return false;
}

void ArmSystem::update()
{
	// Updates the elevator position
	float tmp = targetElevator - encoder_left.getPosition();
	if(tmp > ELEVATOR_THRESHOLD) {
		motor_left.writeMicroseconds(ELEVATOR_UP);
		motor_right.writeMicroseconds(ELEVATOR_UP);
	}
	else if(tmp < -ELEVATOR_THRESHOLD) {
		motor_left.writeMicroseconds(ELEVATOR_DOWN);
		motor_right.writeMicroseconds(ELEVATOR_DOWN);
	}
	else {
		motor_left.writeMicroseconds(MOTOR_BRAKE);
		motor_right.writeMicroseconds(MOTOR_BRAKE);
	}
 	
	// Updates the arm position
 	tmp = targetArm - encoder_arm.getPosition();
	if(tmp > ARM_THRESHOLD) {
		arm.writeMicroseconds(ARM_FOREWARD);
	}
	else if(tmp < -ARM_THRESHOLD) {
		arm.writeMicroseconds(ARM_RETRACT);
	}
	else {
		arm.writeMicroseconds(MOTOR_BRAKE);
	}
 	
	// Updates the claw position
 	tmp = targetClaw - encoder_claw.getPosition();
	if(tmp > CLAW_THRESHOLD) {
		claw.writeMicroseconds(CLAW_OPEN);
	}
	else if(tmp < -CLAW_THRESHOLD) {
		claw.writeMicroseconds(CLAW_CLOSE);
	}
	else {
		claw.writeMicroseconds(MOTOR_BRAKE);
	}
 	
	// Updates the tower position
 	tmp = targetTower - encoder_tower.getPosition();
	if(tmp > TOWER_THRESHOLD) {
		tower.writeMicroseconds(TOWER_RIGHT);
	}
	else if(tmp < -TOWER_THRESHOLD) {
		tower.writeMicroseconds(TOWER_LEFT);
	}
	else {
		tower.writeMicroseconds(MOTOR_BRAKE);
	}
}
