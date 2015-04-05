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
#define TABLE_DISTANCE 30

DriveSystem drive;
ArmSystem arm;
IRSensor ir(A0);
<<<<<<< HEAD
//UltrasonicSensor us_front(2, 3);
//UltrasonicSensor us_top(5, 4);
=======
UltrasonicSensor us_front(3,2);
UltrasonicSensor us_claw(4,5);
>>>>>>> origin/arm_test
long time;
long time_slope;
float minTheta = 0;
<<<<<<< HEAD
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
=======
float ir_last;
int section = 100;
bool hasBottle = false;

PID pid_wall_follow(5,3,30);

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  drive.motor_left.attach(8);
  drive.motor_right.attach(9);

  arm.motor_left.attach(6);
  arm.motor_right.attach(7);
  arm.arm.attach(4);
  arm.claw.attach(3);
  arm.tower.attach(5);

  drive.encoder_right.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
  drive.encoder_left.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  drive.encoder_left.setReversed(false);  // adjust for positive count when moving forward

  arm.encoder_right.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  arm.encoder_right.setReversed(true);  // adjust for positive count when moving forward
  arm.encoder_left.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  arm.encoder_left.setReversed(true);  // adjust for positive count when moving forward
  arm.encoder_arm.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  arm.encoder_arm.setReversed(true);  // adjust for positive count when moving forward
  arm.encoder_claw.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  arm.encoder_claw.setReversed(true);  // adjust for positive count when moving forward
  arm.encoder_tower.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  arm.encoder_tower.setReversed(true);  // adjust for positive count when moving forward

  //calibrate ir sensor
  for(int i = 0; i < 50; i++) {
    ir_last = ir.read();
    us_front.read();
  }

  Serial.println("done setup");
  time = millis();
>>>>>>> origin/arm_test
}

void loop()
{
<<<<<<< HEAD
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
=======
  switch(section)
  {
  case 1:
    {
      Serial.println("Driving");
      // Roberts code for traversing room
      if(!drive.isReady()) {
        us_front.read();
        ir_last = ir.read();
      }
      else if(us_front.read() < WALL_DISTANCE + 5) {
        drive.turnToAngle(1700, PI/2 + 0.5, true);
        pid_wall_follow.reset();
      }
      else if(ir.read() - ir_last > 20 && ir.read() > WALL_DISTANCE) {
        //drive.turn(1800, 300);
      }
      else {
        float error = pid_wall_follow.update(ir.read() - WALL_DISTANCE);
        if(error > 150)
          drive.turn(1700, 150);
        else if (error < -150)
          drive.turn(1700, -150);
        else
          drive.turn(1700, error);
      }
      drive.update();

      //Jai's Code to rudimentary find table
      if(analogRead(A0) < 40) // Calculated threshold by khalil and alex
      {
        Serial.println(analogRead(A3));
        drive.drive(0);
        drive.update();
        if(hasBottle)section = 2;
        else section = 3;
        break;
      }
    }

  case 2:
    {
      Serial.println("At Table");
      arm.setTower(1700);
      arm.setArm(1700);
      arm.setElevator(1700);
      arm.update();

      if(us_claw.read() > 20)
      {
        drive.drive(1700);
      }
      else
      {
       arm.setClaw(1700);
       arm.update();
       arm.setArm(1700);
       arm.update();
       arm.setClaw(1300);
       arm.update();
       arm.setArm(0);
       arm.setTower(0);
       arm.update();
       
       hasBottle = true;
       section = 1;
       break;
      }
    }
    case 100: //debug case
    {
      arm.setElevator(1700);
    }
  }
  
>>>>>>> origin/arm_test
  drive.update();
  
  delay(constrain(100 + time - millis(), 0, 100));
}




