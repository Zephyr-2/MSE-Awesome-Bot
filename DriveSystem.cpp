#include "DriveSystem.h"

DriveSystem::DriveSystem()
{
	targetTheta = 0;
	targetSpeed = MOTOR_BRAKE;
	
	state = STOP;
	lastTime = millis();
}

void DriveSystem::update()
{
	updatePositionAndTheta();
	
	switch(state)
	{
		case DRIVE:
		{
			motor_left.writeMicroseconds(targetSpeed);
			motor_right.writeMicroseconds(targetSpeed);
		}
		break;
		
		case TURN_ANGLE:
		{
			motor_left.writeMicroseconds(constrain(MOTOR_BRAKE, MOTOR_MIN, MOTOR_MAX));
			motor_right.writeMicroseconds(constrain(MOTOR_BRAKE, MOTOR_MIN, MOTOR_MAX));
		}
		break;
		
		case STOP:
		{
			motor_left.writeMicroseconds(MOTOR_BRAKE);
			motor_right.writeMicroseconds(MOTOR_BRAKE);
		}
		break;

		case TURN:
		{
			if(targetTheta >= 0) {
				motor_left.writeMicroseconds(targetSpeed);
				motor_right.writeMicroseconds(map(targetTheta, 0, 1000, targetSpeed, 2*MOTOR_BRAKE-targetSpeed));
			}
			else {
				motor_left.writeMicroseconds(map(targetTheta, 0, -1000, targetSpeed, 2*MOTOR_BRAKE-targetSpeed));
				motor_right.writeMicroseconds(targetSpeed);
			}
		}
	}
}

void DriveSystem::drive(int targetSpeed)
{
	state = DRIVE;
	
	targetTheta = currentTheta;
	this->targetSpeed = constrain(targetSpeed, MOTOR_MIN, MOTOR_MAX);
}

void DriveSystem::brake()
{
	state = STOP;
}

bool DriveSystem::isReady()
{
	if(abs(targetTheta - currentTheta) < ANGLE_THRESHOLD)
		return true;
	return false;
}

void DriveSystem::turnToAngle(int targetSpeed, float targetTheta, bool isRelative)
{
	state = TURN_ANGLE;
	
	this->targetSpeed = constrain(targetSpeed, MOTOR_MIN, MOTOR_MAX);
	this->targetTheta = targetTheta + (isRelative) ? currentTheta : 0;
	constrainTheta(targetTheta);
}

void DriveSystem::turn(int targetSpeed, int turnRadius)
{
	state = TURN;
	this->targetTheta = constrain(turnRadius, -1000, 1000);
	this->targetSpeed = targetSpeed;
}

void DriveSystem::updatePositionAndTheta()
{
	float dist_left = encoder_left.getPosition() - encoder_left_last;
	float dist_right = encoder_right.getPosition() - encoder_right_last;
	float dTheta = (dist_right - dist_left) / CAL_WIDTH;
	float tmp1 = CAL_WIDTH * (dist_right + dist_left) / (dist_right - dist_left) / 2;
	
	currentPosition.x -= tmp1 * (cos(currentTheta) - cos(currentTheta - dTheta));
	currentPosition.y += tmp1 * (sin(currentTheta) - sin(currentTheta - dTheta));
	currentTheta = constrainTheta(currentTheta + dTheta);
}

//constains an angle between -PI and PI
float DriveSystem::constrainTheta(float theta)
{
	while(theta < -PI)
		theta += 2*PI;
	while(theta > PI)
		theta -= 2*PI;
	
	return theta;
}
