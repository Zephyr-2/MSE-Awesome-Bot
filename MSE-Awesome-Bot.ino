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

#define LEFT   0
#define RIGHT  1

#define SCAN_WALL         0
#define SCAN_STOP         1
#define SCAN_TURN         2
#define TIME_DRIVE        3
#define TIME_DRIVE_2      4
#define AT_TABLE          5
#define HAS_BOTTLE        6
#define AT_DOOR           7
#define DRIVE_TO_TMP      8
#define GRAB_BOTTLE       9

#define SCAN_DISTANCE            30
#define SCAN_DISTANCE_THRESHOLD  10
#define SCAN_FAR                 3
#define SCAN_NEAR                1
#define SCAN_FAR_SLOPE           40
#define SCAN_NEAR_SLOPE          100
#define SCAN_STOP_TIME           500
#define SCAN_IR_THRESHOLD        13
#define TABLE_HEIGHT_LOWER       20

DriveSystem drive;
ArmSystem arm;
IRSensor ir(A0);
UltrasonicSensor us_front(3, 4);
UltrasonicSensor us_side(2, 2);

int state;

long time;
long doorDriveTime;
long timer;
long drive_timer;
long last_light, current_light, average_light(0), sum_light;
int lightcount = 0;
float minTheta = 0;
float tmp;

int turnDir, nextState;

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
  
  current_light = analogRead(A1);
  last_light = current_light;

  state = SCAN_WALL;
}

void loop()
{
  time = millis();

  ir.read();
  us_front.read();
  us_side.read();

  current_light = analogRead(A1);
  
  sum_light += current_light;
  sum_light = sum_light; 
  lightcount++;
  
  if(lightcount > 20)
  {
  average_light = sum_light/lightcount;
  sum_light = 0;
  lightcount = 0;
  Serial.println(average_light);

<<<<<<< HEAD
=======
  }
  
  float tmp;

>>>>>>> origin/arm_test
  switch(state) {
  case SCAN_WALL:
    if(!arm.towerAtPosition() || !arm.towerAtPosition() || !arm.armAtPosition())
      break;
<<<<<<< HEAD
    //put light detection code here
    if (current_light > last_light && last_light < 180 && us_side.current() < 10)
=======
    if (current_light > last_light && last_light < 80 && us_side.current() < 10)
>>>>>>> origin/arm_test
    {
      turn(RIGHT, TIME_DRIVE_2);
      break; 
    }
    else if (current_light > last_light && last_light < 180 && !arm.hasBottle)
    {
      drive.brake();
      arm.setTower(90);
      arm.setClaw(90);
      state = AT_TABLE;
      break;
    }
    else
    {
      //arm.setElevator(75);
    }


    if(usInFront()) {
      turn(LEFT, SCAN_WALL);
      break;
    }

    if(usOffSide()) {
      state = TIME_DRIVE;
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

  case AT_DOOR:

    drive.turnToAngle(1700, -PI/2, 1);
	doorDriveTime = time;
	if((time-doorDriveTime)<3000)
	{
	   drive.drive(1700);
	}
	state = SCAN_WALL;
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

  case AT_TABLE:
    if(!arm.towerAtPosition() || !arm.clawAtPosition())
      drive.turn(1500, 0);
    else
      drive.turn(1400, 0);
    
    if(ir.last() < ir.current() && ir.last() < 50) {
      tmp = drive.encoder_left.getPosition() + 4;
      state = DRIVE_TO_TMP;
    }
    break;
    
    case DRIVE_TO_TMP:
    Serial.println(drive.encoder_left.getPosition());
    Serial.println(tmp);
    Serial.println(" ");
      if(drive.encoder_left.getPosition() > tmp)
      {
        arm.setArm(90);
        state = GRAB_BOTTLE;
      }

      drive.turn(1650, 0);
    break;
    
    case GRAB_BOTTLE:
      drive.brake();
      
      if(arm.armAtPosition())
        arm.setClaw(0);
      
      if(arm.encoder_claw.getPosition() < 50) {
        arm.setTower(0);
        arm.setArm(0);
        state = SCAN_WALL;
        arm.hasBottle = true;
      }
    break;
  }

  arm.update();
  drive.update();
  last_light = current_light;

  delay(constrain(100 + time - millis(), 0, 100));
}

bool usInFront() {
  return us_front.current() < SCAN_DISTANCE && ir.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD;
}

bool usOffSide() {
  return us_side.current() > 2 * SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD;
}

bool irInFront() {
  return ir.current() < SCAN_IR_THRESHOLD;
}

void turn(bool dir, int next) {
  drive.brake();
  timer = millis() + SCAN_STOP_TIME;
  turnDir = dir;
  nextState = next;
  state = SCAN_STOP;
}


