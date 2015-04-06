
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

#define SCAN_FOLLOW_WALL  0
#define SCAN_STOP         1
#define SCAN_TURN         2
<<<<<<< HEAD
#define TIME_DRIVE        3
#define TIME_DRIVE_2      4
=======
#define AT_TABLE          3
#define HAS_BOTTLE        4
#define AT_DOOR           5
>>>>>>> origin/arm_test

#define SCAN_DISTANCE            50
#define SCAN_DISTANCE_THRESHOLD  10
#define SCAN_FAR                 3
#define SCAN_NEAR                1
#define SCAN_FAR_SLOPE           40
#define SCAN_NEAR_SLOPE          100
<<<<<<< HEAD
#define SCAN_STOP_TIME           500
#define SCAN_IR_THRESHOLD        13
#define TABLE_HEIGHT_LOWER       20
=======
#define SCAN_STOP_TIME           50
>>>>>>> origin/arm_test

DriveSystem drive;
ArmSystem arm;
IRSensor ir(A0);
UltrasonicSensor us_front(3, 4);
UltrasonicSensor us_side(2, 2);

Point table1;
Point door1;
Point table2;
Point door2;

int state;

long time;
long timer;
long drive_timer;
float minTheta = 0;

bool firstrun = true;

void setup()
{
<<<<<<< HEAD
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
        drive_timer = -1;
        
        state = SCAN_WALL;
=======
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

  state = SCAN_FOLLOW_WALL;
>>>>>>> origin/arm_test
}

void loop()
{
  time = millis();

  ir.read();
  us_front.read();
  us_side.read();
<<<<<<< HEAD
  
=======

>>>>>>> origin/arm_test
  float tmp;

  switch(state) {
<<<<<<< HEAD
    case SCAN_WALL:
      //put light detection code here
      
      if(usInFront()) {
        turn(LEFT, SCAN_WALL);
        break;
      }
      
      if(usOffSide()) {
        turn(RIGHT, TIME_DRIVE);
        break;
      }
      
      if(us_side.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD)
        tmp = (us_side.slope() + SCAN_FAR) * SCAN_FAR_SLOPE;
      else if(us_side.current() < SCAN_DISTANCE - SCAN_DISTANCE_THRESHOLD)
        tmp = (us_side.slope() - SCAN_FAR) * SCAN_FAR_SLOPE;
      else if(abs(us_side.slope()) > SCAN_NEAR)
        tmp = us_side.slope() * SCAN_NEAR_SLOPE;
       else
        tmp = 0;
      
      if(tmp > 200)
        tmp = 200;
      else if(tmp < -200)
        tmp = -200;
        
      drive.turn(1700, tmp);
    break;
    
    case SCAN_STOP:
      if(millis() > timer) {
        drive.turnToAngle(1700, (turnDir) ? -PI/2 : PI/2, true);
        state = SCAN_TURN;
      }
    break;
    
    case SCAN_TURN:
      if(drive.isReady() && arm.towerAtPosition())
        state = nextState;
    break;
    
    case TIME_DRIVE:
    
      if(drive_timer == -1)
        drive_timer = millis() + 1000;
        
      if(millis() > drive_timer) {
        turn(RIGHT, TIME_DRIVE_2);
        drive_timer = -1;
        break;
      }
      
      drive.turn(1700, 0);
     break;
     
     case TIME_DRIVE_2:
         if(drive_timer == -1)
          drive_timer = millis() + 1000;
          
         if(millis() > drive_timer) {
          state = SCAN_WALL;
          drive_timer = -1;
          break;
        }
        
        drive.turn(1700, 0);
      break;
=======
  case SCAN_FOLLOW_WALL:
    //put light code here
    Serial.println(analogRead(A1));
    if ((analogRead(A1) < 40) && (us_side.current() < 10))
    {
      drive.brake();
      drive.drive(1700);
      state = AT_DOOR;
      break; 
    }
    else if (analogRead(A1) < 40)
    {
      drive.brake();
      arm.setTower(90);
      Serial.println(arm.encoder_tower.getPosition());
      state = AT_TABLE;
      break;
    }
    else
    {
      arm.setElevator(75);
    }

    if(us_front.current() < SCAN_DISTANCE + 10 && (ir.current() < 10 || ir.current() > 65)) {
      drive.brake();
      timer = millis() + SCAN_STOP_TIME;
      state = SCAN_STOP;
      return;
    }

    if(us_side.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD)
      tmp = (us_side.slope() + SCAN_FAR) * SCAN_FAR_SLOPE;
    else if(us_side.current() < SCAN_DISTANCE - SCAN_DISTANCE_THRESHOLD)
      tmp = (us_side.slope() - SCAN_FAR) * SCAN_FAR_SLOPE;
    else if(abs(us_side.slope()) > SCAN_NEAR)
      tmp = us_side.slope() * SCAN_NEAR_SLOPE;
    else
      tmp = 0;

    if(tmp > 200)
      tmp = 200;
    else if(tmp < -200)
      tmp = -200;

    drive.turn(1700, tmp);
    break;

  case SCAN_STOP:
    if(millis() > timer) {
      drive.turnToAngle(1700, PI/2, true);
      state = SCAN_TURN;
    }
    break;

  case SCAN_TURN:
    if(drive.isReady())
      state = SCAN_FOLLOW_WALL;
    break;

  case AT_TABLE:
    Serial.println(ir.current());
    if(ir.current() < 65){
      drive.drive(1700);
      arm.setArm(20);
      arm.setClaw(100);
      if(ir.current() < 50){
        drive.brake();
        arm.setArm(100);
        arm.setClaw(0);
        state = HAS_BOTTLE;
      }
    }
    break;

>>>>>>> origin/arm_test
  }

  arm.update();
  drive.update();

  delay(constrain(100 + time - millis(), 0, 100));
}

<<<<<<< HEAD
bool usInFront() {
  return us_front.current() < SCAN_DISTANCE && ir.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD;
}

bool usOffSide() {
  return us_side.current() > SCAN_DISTANCE + 2 * SCAN_DISTANCE_THRESHOLD;
}
=======

>>>>>>> origin/arm_test



