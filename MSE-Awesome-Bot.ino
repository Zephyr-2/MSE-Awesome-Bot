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

#define WALL_DISTANCE  80

DriveSystem drive;
ArmSystem arm;
IRSensor ir(A0);
//UltrasonicSensor us_front(2, 3);
//UltrasonicSensor us_top(5, 4);
long time;
long time_slope;
float minTheta = 0;
float ir_current, ir_last;
float ir_slope;

void setup()
{
	Serial.begin(9600);
	Wire.begin();

	drive.motor_left.attach(9);
	drive.motor_right.attach(8);

	drive.encoder_right.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
	drive.encoder_left.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_left.setReversed(false);  // adjust for positive count when moving forward

        arm.motor_left.attach(11);
        arm.motor_right.attach(12);
        arm.arm.attach(7);
        arm.claw.attach(6);
        arm.tower.attach(10);
        
        ir_current = 20;
        ir_last = 20;
        
        arm.encoder_tower.init(21.4 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
        arm.encoder_tower.setReversed(false);  // adjust for positive count when moving forward
        arm.encoder_right.init(15.574 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
        arm.encoder_right.setReversed(true);  // adjust for positive count when moving forward
        arm.encoder_left.init(15.574 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
        arm.encoder_left.setReversed(false);  // adjust for positive count when moving forward
        arm.encoder_arm.init(10.989 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
        arm.encoder_arm.setReversed(false);  // adjust for positive count when moving forward
        arm.encoder_claw.init(43.478 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
        arm.encoder_claw.setReversed(false);  // adjust for positive count when moving forward

        Serial.println("done setup");
	time = millis();
        time_slope = millis();
}

void loop()
{
  time = millis();
  ir_current = ir.read();
  
  if(millis() - time_slope > 500) {
    ir_slope = 2 * (ir_current - ir_last);
    time_slope = millis();
    ir_last = ir_current;
  }
  
  Serial.println(ir_current);
  Serial.println(ir_last);
  Serial.println(ir_slope);
  Serial.println(" ");
  
  if(ir.read() > 70) {
    if(ir_slope < -5)
      drive.turn(1800, 0);
    else
      drive.turn(1800, 200);
  }
  else if(ir.read() < 60) {
    if(ir_slope > 5)
      drive.turn(1800, 0);
    else
      drive.turn(1800, -200);
  }
  else {
    if(ir_slope > 2)
      drive.turn(1800, 200);
    else if(ir_slope < -2)
      drive.turn(1800, -200);
    else
      drive.turn(1800, 0);
  }
     
  arm.update();
  drive.update();
  
  delay(constrain(100 + time - millis(), 0, 100));
}
