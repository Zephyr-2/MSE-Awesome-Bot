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

DriveSystem drive;
ArmSystem arm;
UltrasonicSensor us_left(9, 10);
UltrasonicSensor us_right(11, 12);
IRSensor ir(1);
long time;
int section = 0;
float findhighest = 0;

//Point Variables
Point firstroomcoordinates[4];
Point secondroomcoordinates[4];
Point largetablecoordinates[4];
Point doorcoordinate;
Point smalltablecoordinates[4];
Point recyclecoordinate;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  drive.motor_left.attach(2);
  drive.motor_right.attach(8);

  drive.encoder_right.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
  drive.encoder_left.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
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
  //section = 9;
  switch(section)
  {  
    case(0):
    {
      // Scan First Room and Find Corners
      drive.turn(1600,90);
      drive.update();
      if (drive.getTheta() != 90)
      {
        if(findhighest < ir.read())
        {
         findhighest = ir.read(); 
        }
      }
      if(drive.getTheta() == 90)
      {
        //something here about point
      }
      section = 1;
    };
    case(1):
    {
      // Find Table
      
      section = 2;
    };
    case(2):
    {
      // Approach Table
      if((drive.getPosition().x != largetablecoordinates[1].x) && (drive.getPosition().y != largetablecoordinates[1].y))
      {
        drive.update();
      }
      section = 3;
    };
    case(3):
    {
      // Move Alongside Table and Pickup Waterbottle
      section = 4;
    };
    case(4):
    {
      // Move to door
      section = 5;
    };
    case(5):
    {
      // Go through door
      section = 6;
    };
    case(6):
    {
      // Scan Second Room and Find Corners
      section = 7;
    };
    case(7):
    {
      // Find Table
      section = 8;
    };
    case(8):
    {
      // Approach Table
      section = 9;
      // Return Waterbottle on back corner
    };
    case(9):
    {
      // Pickup Empty Waterbottle
      
      
      
      section = 10;
    };
    case(10):
    {
      // Go to Door
      section = 11;
    };
    case(11):
    {
      // Move through Door
      section = 12;
    };
    case(12):
    {
      // Return to Table 1
      section = 13;
    };
    case(13):
    {
      // Return Empty Waterbottle
      section = 14;
    };
  }

  delay(10);
}


