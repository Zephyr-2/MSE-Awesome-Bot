#include "DriveSystem.h"

DriveSystem::DriveSystem() : pid_drive(1,0,0), pid_turn_angle(1,0,0)
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
			//uses a PID to try and match the current angle to the set angle
			float offset = pid_drive.update(currentTheta - targetTheta);
			
			if(offset > 0)
			{
				motor_left.writeMicroseconds(targetSpeed);
				motor_right.writeMicroseconds(targetSpeed - (int) offset);
			}
			else
			{
				motor_left.writeMicroseconds(targetSpeed - (int) offset);
				motor_right.writeMicroseconds(targetSpeed);
			}
		}
		break;
		
		case TURN:
		{
			float angle_offset = pid_turn_angle.update(constrainTheta(targetTheta - currentTheta));
			angle_offset *= targetSpeed - MOTOR_BRAKE;

			motor_left.writeMicroseconds(constrain(MOTOR_BRAKE - angle_offset, MOTOR_MIN, MOTOR_MAX));
			motor_right.writeMicroseconds(constrain(MOTOR_BRAKE + angle_offset, MOTOR_MIN, MOTOR_MAX));
		}
		break;
		
		case STOP:
		{
			motor_left.writeMicroseconds(MOTOR_BRAKE);
			motor_right.writeMicroseconds(MOTOR_BRAKE);
		}
		break;
	}
}

void DriveSystem::drive(int targetSpeed)
{
	state = DRIVE;
	pid_drive.reset();
	
	targetTheta = currentTheta;
	this->targetSpeed = constrain(targetSpeed, MOTOR_MIN, MOTOR_MAX);
}

void DriveSystem::brake()
{
	state = STOP;
}

void DriveSystem::turn(int targetSpeed, float targetTheta)
{
	state = TURN;
	pid_turn_angle.reset();
	
	this->targetSpeed = constrain(targetSpeed, MOTOR_MIN, MOTOR_MAX);
	this->targetTheta = targetTheta;
	constrainTheta(targetTheta);
	
	encoder_left.zero();
	encoder_right.zero();
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
