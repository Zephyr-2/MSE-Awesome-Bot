#include "ArmSystem.h"

ArmSystem::ArmSystem() : pid_elevator(1,0,0)
{
	targetElevator = 0;
	targetArm = 0;
	targetClaw = 0;

	encoder_left.zero();
	encoder_right.zero();
	encoder_arm.zero();
	encoder_claw.zero();
}

void ArmSystem::setElevator(int targetElevator)
{
	this->targetElevator = targetElevator;
	pid_elevator.reset();
}

void ArmSystem::setArm(int targetArm)
{
	this->targetArm = targetArm;
}

void ArmSystem::setClaw(int targetClaw)
{
	this->targetClaw = targetClaw;
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

void update()
{
	float tmp = targetElevator - encoder_left.getPosition();
	if(abs(tmp) > THRESHOLD) {
		left_elevator.writeMicroseconds(constrain(MOTOR_BRAKE + tmp * 200));
		right_elevator.writeMicroseconds(elevator_speed);
	}
	else {
		left_elevator.writeMicroseconds(MOTOR_BRAKE);
		right_elevator.writeMicroseconds(MOTOR_BRAKE);
	}

	int arm_speed = 
}