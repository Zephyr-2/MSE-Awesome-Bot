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
UltrasonicSensor us_front(3,2);
UltrasonicSensor us_claw(4,5);
long time;
int phase = 0;
float minTheta = 0;
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
}

void loop()
{
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
  
  drive.update();
  delay(constrain(100 + time - millis(), 0, 100));
}




