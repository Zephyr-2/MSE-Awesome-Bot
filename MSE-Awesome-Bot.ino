#include "DriveSystem.h"
#include "ArmSystem.h"
#include "IRSensor.h"
#include "PID.h"
#include "Point.h"
#include "UltrasonicSensor.h"
#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

#define ARM_LOWER
#define ARM_UPPER

DriveSystem drive;
ArmSystem arm;
UltrasonicSensor us_left(9, 10);
UltrasonicSensor us_right(11, 12);
IRSensor ir(A1);
long time;

void setup()
{
	Serial.begin(115200);
	Wire.begin();

	drive.motor_left.attach(2);
	drive.motor_right.attach(8);

	drive.encoder_right.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
	drive.encoder_left.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_left.setReversed(true);  // adjust for positive count when moving forward

	arm.motor_left.attach(3);
	arm.motor_right.attach(4);
	arm.arm.attach(5);
	arm.claw.attach(6);

	arm.encoder_right.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	arm.encoder_right.setReversed(true);  // adjust for positive count when moving forward
	arm.encoder_left.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	arm.encoder_left.setReversed(true);  // adjust for positive count when moving forward
	arm.encoder_arm.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	arm.encoder_arm.setReversed(true);  // adjust for positive count when moving forward
	arm.encoder_claw.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	arm.encoder_claw.setReversed(true);  // adjust for positive count when moving forward

	time = millis();
}

void loop()
{
	time = millis();

	if(ir.read() < 50)
	{
                drive.turn(1700,250);
	}
	else if(ir.read() > 50)
	{
		drive.turn(1700,-250);
	}
	
Serial.println("run");
	delay(constrain(100 + time - millis(), 0, 100));
}
