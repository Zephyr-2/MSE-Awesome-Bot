/**
 * The main logic brach of the robot code. This brach will
 * make the robot follow around the room, pick up the water
 * bottle, drive throuhg the door, place the water bottle, pick
 * up an empty water bottle, and drive back through the room to
 * recycle the water bottle
 * 
 * Author: Robert Meagher, Jai Sood and Alex Yuan
 * Written for MSE2202B 2015
 */ 

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

// Used to define turns
#define LEFT 						0
#define RIGHT 						1

// List of possible robot states
#define SCAN_WALL 					0
#define SCAN_STOP 					1
#define SCAN_TURN 					2
#define TIME_DRIVE 					3
#define TIME_DRIVE_2 				4
#define AT_TABLE_FRONT				5
#define AT_TABLE_BACK				6
#define DRIVE_TO_TMP_F				7
#define DRIVE_TO_TMP_B				8
#define GRAB_BOTTLE					9
#define TIME_DRIVE_DOOR 			10
#define TIME_DRIVE_DOOR_2			11
#define DROP_TABLE_BACK				12
#define DROP_TABLE_FRONT			13
#define TIME_REVERSE				14

// Calibration constants
#define SCAN_DISTANCE 				35
#define SCAN_DISTANCE_THRESHOLD 	5
#define SCAN_FAR 					3
#define SCAN_NEAR 					1
#define SCAN_FAR_SLOPE				40
#define SCAN_NEAR_SLOPE				100
#define SCAN_STOP_TIME				500
#define SCAN_IR_THRESHOLD			13
#define TABLE_HEIGHT_LOWER			20
#define STEADY_LIGHT				1.6

DriveSystem drive;
ArmSystem arm;
IRSensor ir(A0);
UltrasonicSensor us_front(3, 4);
UltrasonicSensor us_side(2, 2);

int buttonPin = 5;	// The pin attached to the button
int lightPin = A1;	// The pin attached to the light sensor

long time;
long drive_timer;

long current_light;

long average_light;
long sum_average_light;
long count_average_light;

long ambient_light;
long sum_ambient_light;
long count_ambient_light;

float tmp;
int turnDir;
bool done_in_room;

int state;			// The current state of the robot
int nextState;		// The next state of the robot

/**
 * Initializes all robot systems and tells the robot to begin moving
 * around the room
 */
void setup()
{
	Serial.begin(9600);
	Wire.begin();

	initDrive();
	initArm();

	pinMode(buttonPin, INPUT_PULLUP);
	
	pinMode(lightPin, INPUT);
	ambient_light = 0.5 * average_light;

	drive_timer = -1;
	state = SCAN_WALL;
	done_in_room = false;
	time = millis();

	Serial.println("done setup");
}

/**
 * Initializes the drive motors and encoders
 * Encoder values are measured in cm and cm/s
 */
void initDrive()
{
	drive.motor_left.attach(9);
	drive.motor_right.attach(8);

	drive.encoder_right.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
	drive.encoder_left.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_left.setReversed(false);  // adjust for positive count when moving forward
}

/**
 * Initializes the arm motors and encoders, and sets the elevator
 * to its initial height.
 * 
 * Encoder values are as follows:
 * Tower: 0 is foreward, 90 is 90 degrees clockwise, -90 is 90 degrees counter-clockwise
 * Arm, Elevator and Claw: 0 is minimum position, 100 is maximum position
 */
void initArm()
{
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

	arm.setElevator(100);
}

/**
 * The main loop of the robot code.
 */
void loop()
{
	time = millis();

	// Read sensor values
	ir.read();
	us_front.read();
	us_side.read();

	// Update the average and ambient light values
	sum_average_light += analogRead(lightPin); 
	count_average_light += 100;
	sum_ambient_light += analogRead(lightPin); 
	count_ambient_light += 100;

	// If a second has passed, update the average light reading
	if(count_average_light > 1000) {
		average_light = sum_average_light/ 10;
		sum_average_light = 0;
		count_average_light = 0;
	}

	// If 2.5 seconds have passed, update the ambient light reading
	if(count_ambient_light > 2500) {
		ambient_light = sum_ambient_light / 25;
		sum_ambient_light = 0;
		count_ambient_light = 0;
	}

	switch(state) {
		case SCAN_WALL:
		// If the button is pressed, reverse and turn left
		if(!digitalRead(buttonPin)) {
			state = TIME_REVERSE;
			break;
		}

		// If the arm system is not at its target values, exit
		if(!arm.elevatorAtPosition() || !arm.towerAtPosition() || !arm.armAtPosition())
		break;

		// If the robot sees a steady light and has the water bottle, drive through the door
		if ((float) average_light / ambient_light < STEADY_LIGHT && done_in_room) {
			state = TIME_DRIVE;
			done_in_room = false;
			arm.setElevator(0);
			break;
		}
		// If the robot sees a steady light and does not have a water bottle
		else if ((float) average_light / ambient_light < STEADY_LIGHT && !done_in_room) {
			drive.brake();
			arm.setTower(90);

			// The the robot already has a bottle, drop it and pick up a water bottle
			if(arm.hasBottle) {
				state = DROP_TABLE_BACK;
				arm.setArm(40);
			}
			// If the robot does not have a water bottle, pick it up
			else {
				state = AT_TABLE_BACK;
				arm.setClaw(90);
			}
			break;
		}

		// If an object is detected in front of the robot, turn left
		if(usInFront()) {
			turn(LEFT, SCAN_WALL);
			break;
		}

		// If the robot has lost sight of what is beside it, turn right
		if(usOffSide()) {
			state = TIME_DRIVE;
			break;
		}

		// Turns the robot to be perpendicular from the wall
		if(us_side.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD)
			tmp = (us_side.slope() + SCAN_FAR) * SCAN_FAR_SLOPE;
		else if(us_side.current() < SCAN_DISTANCE - SCAN_DISTANCE_THRESHOLD)
			tmp = (us_side.slope() - SCAN_FAR) * SCAN_FAR_SLOPE;
		else if(abs(us_side.slope()) > SCAN_NEAR)
			tmp = us_side.slope() * SCAN_NEAR_SLOPE;
		else
			tmp = 0;

		// Sets a maximum turn value for the robot
		if(tmp > 200)
			tmp = 200;
		else if(tmp < -200)
			tmp = -200;

		drive.turn(1800, tmp);
		break;

		// Stops the robot to allow it to turn more accurately
		case SCAN_STOP:
		if(millis() > drive_timer) {
			drive_timer = -1;
			drive.turnToAngle(1700, (turnDir) ? -PI/2 - 0.1 : PI/2 + 0.1, true);
			state = SCAN_TURN;
		}
		break;

		// Waits until the robot has finished turning
		case SCAN_TURN:
		if(drive.isReady() && arm.towerAtPosition())
			state = nextState;
		break;

		// Drives the robot foreward to avoid hitting an obstacle before turning right
		// Followed by TIME_DRIVE_2
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

		// Drives the robot foreward to allow it to detect an obstacle before resuming scan
		case TIME_DRIVE_2:
		if(drive_timer == -1)
		drive_timer = millis() + 2000;

		if(millis() > drive_timer) {
			state = SCAN_WALL;
			drive_timer = -1;
			break;
		}

		drive.turn(1700, 0);
		break;

		// Reverses the robot and scans the table for the water bottle
		case AT_TABLE_FRONT:
		if(!arm.towerAtPosition() || !arm.clawAtPosition())
		drive.turn(1500, 0);
		else
		drive.turn(1400, 0);

		// If the robot sees the bottle, pick it up
		if(ir.current() - ir.last() >  10 && ir.last() < 30) {
			tmp = drive.encoder_left.getPosition() + 4;
			state = DRIVE_TO_TMP_F;
		}
		break;

		// Drives the robot foreward and scans the table for the water bottle
		case AT_TABLE_BACK:
		if(!arm.towerAtPosition() || !arm.clawAtPosition())
		drive.turn(1500, 0);
		else
		drive.turn(1600, 0);

		// If the robot sees the bottle, pick it up
		if(ir.current() - ir.last() >  10 && ir.last() < 40) {
			tmp = drive.encoder_left.getPosition() - 4;
			state = DRIVE_TO_TMP_F;
		}
		break;

		// Slowly drives the robot foreward to line it up with the bottle
		case DRIVE_TO_TMP_F:
		if(drive.encoder_left.getPosition() > tmp)
		{
			arm.setArm(90);
			state = GRAB_BOTTLE;
		}

		drive.turn(1650, 0);
		break;

		// Slowly drives the robot backward to line it up with the bottle
		case DRIVE_TO_TMP_B:
		if(drive.encoder_left.getPosition() > tmp)
		{
			arm.setArm(90);
			state = GRAB_BOTTLE;
		}

		drive.turn(1350, 0);
		break;

		// Grabs the bottle, then retracts the arm and lowers and straigtens
		// the tower
		case GRAB_BOTTLE:
		drive.brake();

		if(arm.armAtPosition())
		arm.setClaw(0);

		if(arm.encoder_claw.getPosition() < 50) {
			arm.setArm(0);

			if(arm.armAtPosition()) {
				arm.setTower(0);

				if(arm.towerAtPosition()) {
					state = SCAN_WALL;
					arm.hasBottle = true;
					done_in_room = true;
				}
			}
		}
		break;

		// Drops the bottle and searches for a new bottle in front of it
		case DROP_TABLE_BACK:
		if(arm.armAtPosition()) {
			arm.setClaw(90);
			arm.setArm(0);
			arm.setTower(95);
			state = AT_TABLE_BACK;
		}

		drive.turn(1500, 0);
		break;

		// Drops the bottle and searches for a new bottle in behind it
		case DROP_TABLE_FRONT:
		if(arm.armAtPosition()) {
			arm.setClaw(90);
			arm.setArm(0);
			arm.setTower(95);
			state = AT_TABLE_FRONT;
		}

		drive.turn(1500, 0);
		break;

		// Reverses the robot to prevent obstacle collision when turning
		case TIME_REVERSE:
		if(drive_timer == -1)
		drive_timer = millis() + 1000;

		if(millis() > drive_timer) {
			turn(LEFT, SCAN_WALL);
			drive_timer = -1;
			break;
		}

		drive.turn(1300, 0);
		break;

		// Lines the robot up with the door
		case TIME_DRIVE_DOOR:
		arm.setElevator(0);
		drive.turn(1700, 0);

		if(drive_timer == -1)
		drive_timer = millis() + 1000;

		if(millis() > drive_timer) {
			drive.brake();

			if(arm.elevatorAtPosition()) {
				turn(RIGHT, TIME_DRIVE_DOOR_2);
				drive_timer = -1;
			}
			break;
		}
		break;

		// Drives the robot through the door
		case TIME_DRIVE_DOOR_2:
		if(drive_timer == -1)
		drive_timer = millis() + 3000;

		if(millis() > drive_timer) {
			state = SCAN_WALL;
			drive_timer = -1;
			break;
		}

		drive.turn(2100, 0);
		break;
	}

	arm.update();
	drive.update();

	// Delays the loop to create a 0.1s update time.
	delay(constrain(100 + time - millis(), 0, 100));
}

/**
 * Returns true if the robot senses an object in front of itself
 */
bool usInFront() {
	return (us_front.current() < SCAN_DISTANCE && ir.current() > SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD) || (us_front.current() < 20 && ir.current() < 30);
}

/**
 * Returns true is the side ultrasonic has lost sight of the wall
 */
bool usOffSide() {
	return us_side.current() > 3 * SCAN_DISTANCE + SCAN_DISTANCE_THRESHOLD;
}

/**
 * Returns true if an object is in front of the infrared sensor
 */
bool irInFront() {
	return ir.current() < SCAN_IR_THRESHOLD;
}

/**
 * Turns the robot and sets its next command
 *
 * @param	dir 	The direction to turn (left/right)
 * @param	next 	The state to switch to after the turn is complete
 */
void turn(bool dir, int next) {
	drive.brake();
	drive_timer = millis() + SCAN_STOP_TIME;
	turnDir = dir;
	nextState = next;
	state = SCAN_STOP;
}