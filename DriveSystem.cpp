#include "DriveSystem.h"

DriveSystem::DriveSystem()
{
        currentPosition.x = 0;
        currentPosition.y = 0;
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
                        if(abs(targetTheta - currentTheta) < ANGLE_THRESHOLD) {
                          motor_left.writeMicroseconds(MOTOR_BRAKE);
			  motor_right.writeMicroseconds(MOTOR_BRAKE);
                        }
                        else if(targetTheta > currentTheta) {
                          motor_left.writeMicroseconds(2*MOTOR_BRAKE - targetSpeed);
                          motor_right.writeMicroseconds(targetSpeed);
                        }
                        else {
                          motor_left.writeMicroseconds(targetSpeed);
                          motor_right.writeMicroseconds(2*MOTOR_BRAKE - targetSpeed);
                        }
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
	if(state == TURN_ANGLE && abs(targetTheta - currentTheta) < ANGLE_THRESHOLD)
		return true;
	return false;
}

void DriveSystem::turnToAngle(int targetSpeed, float targetTheta, bool isRelative)
{
	state = TURN_ANGLE;
        Serial.println(targetTheta);
        Serial.println(currentTheta);
        Serial.println(" ");
	
	this->targetSpeed = constrain(targetSpeed, MOTOR_MIN, MOTOR_MAX);
	this->targetTheta = targetTheta;
        if(isRelative) targetTheta += currentTheta;
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

        if(dTheta != 0) {
          float tmp1 = CAL_WIDTH * (dist_right + dist_left) / (dist_right - dist_left) / 2;
	  currentPosition.x += tmp1 * (cos(currentTheta) - cos(currentTheta - dTheta));
	  currentPosition.y += tmp1 * (sin(currentTheta) - sin(currentTheta - dTheta));
        }
        else
        {
          currentPosition.x -= dist_left * cos(currentTheta);
	  currentPosition.y += dist_left * sin(currentTheta);
        }
        
	currentTheta += dTheta;
        encoder_left_last = encoder_left.getPosition();
        encoder_right_last = encoder_right.getPosition();
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
