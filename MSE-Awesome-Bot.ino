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
#define SCAN_TO_WALL      3
#define SCAN_IR           4

#define SCAN_DISTANCE            70
#define SCAN_DISTANCE_THRESHOLD  5
#define SCAN_FAR                 3
#define SCAN_NEAR                1
#define SCAN_FAR_SLOPE           40
#define SCAN_NEAR_SLOPE          100
#define SCAN_STOP_TIME           50
#define SCAN_IR_THRESHOLD        20
#define TABLE_HEIGHT_LOWER       20

DriveSystem drive;
ArmSystem arm;
IRSensor ir(A3);
UltrasonicSensor us_front(3, 4);
UltrasonicSensor us_side(2, 2);

int state;

long time;
long timer;
float minTheta = 0;

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
        
        state = SCAN_TO_WALL;
}

void loop()
{
  time = millis();

  ir.read();
  us_front.read();
  us_side.read();
  
  Serial.println(analogRead(A3));
  
  float tmp;
  
  switch(state) {
    case SCAN_WALL:
      //put light detection code here
      
      if(usInFront()) {
        turn(LEFT, SCAN_WALL);
        return;
      }
      
      if(usSideChange()) {
        turn(RIGHT, SCAN_TO_WALL);
        return; 
      }
      
      if(irInFront()) {
        arm.setTower(90);
        turn(LEFT, SCAN_IR);
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
        drive.turnToAngle(1700, (turnDir) ? PI/2 : -PI/2, true);
        state = SCAN_TURN;
      }
    break;
    
    case SCAN_TURN:
      if(drive.isReady() && arm.towerAtPosition())
        state = nextState;
    break;
    
    case SCAN_TO_WALL:
      if(usInFront()) {
        turn(LEFT, SCAN_WALL);
        return;
      }
      
      if(irInFront()) {
        arm.setTower(90);
        turn(LEFT, SCAN_IR);
        return;
      }
      
      drive.turn(1700, 0);
    break;
    
    case SCAN_IR:
      if(usInFront()) {
        arm.setTower(0);
        turn(LEFT, SCAN_WALL);
        return;
      }
      
      if(!irInFront()) {
        turn(RIGHT, SCAN_IR);
      }
      
      drive.turn(1700, 0);
    break;
  }
  
  arm.update();
  drive.update();
  
  delay(constrain(100 + time - millis(), 0, 100));
}

bool usInFront() {
  return us_front.current() < SCAN_DISTANCE + 10 && ir.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD;
}

bool usSideChange() {
  return us_side.slope() > 10 && us_side.current() > SCAN_DISTANCE + 2 * SCAN_DISTANCE_THRESHOLD;
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

