/**
 * Controls the arm motors of the robot, including
 * the rotating tower, the elevator, the arm motor
 * and the claw motor.
 *
 * Author: Robert Meagher and Jai Sood
 * Written for MSE2202B 2015
 */

#ifndef ARMSYSTEM_H
#define ARMSYSTEM_H

#include "Arduino.h"
#include "PID.h"
#include <Servo.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>
#include <Wire.h>

#define MOTOR_BRAKE				1500

#define ELEVATOR_UP				1950
#define ELEVATOR_DOWN			1300
#define ELEVATOR_THRESHOLD		2

#define ARM_FOREWARD			1800
#define ARM_RETRACT				1200
#define ARM_THRESHOLD			5

#define CLAW_CLOSE				1300
#define CLAW_OPEN				1700
#define CLAW_THRESHOLD			15

#define TOWER_LEFT				1700
#define TOWER_RIGHT				1300
#define TOWER_THRESHOLD			7

class ArmSystem
{
public:
	/**
	 * Initializes the arm system, and sets all motors to minimm values
	 */
	ArmSystem();
	
	/**
	 * Sets the elevtor to a target elevation, in percent
	 *
	 * @param targetElevation	The elevation to set the elevator to
	 */
	void setElevator(int targetElevation);

	/**
	 * Sets the arm to a target extension, in percent
	 *
	 * @param targetLength	The percent extension to set the arm to
	 */
	void setArm(int targetlength);

	/**
	 * Sets the claw to a target opening, in percent
	 *
	 * @param targetClaw	The percent to set the claw to
	 */
	void setClaw(int targetClaw);

	/**
	 * Sets the tower to a target rotation, in degrees
	 * Positive is clockwise, negative is counter-clockwise
	 *
	 * @param targetTheta	The rotation to set the tower to
	 */
	void setTower(int targetTheta);
	
	/**
	 * Whether the elevator is at its target height
	 *
	 * @return	True if the elevator is at its target height
	 */
	bool elevatorAtPosition();

	/**
	 * Whether the arm is at its target extension
	 *
	 * @return	True if the arm is at its target extension
	 */
	bool armAtPosition();

	/**
	 * Whether the claw is at its target position
	 *
	 * @return	True if the claw is at its target position
	 */
	bool clawAtPosition();

	/**
	 * Whether the tower is at its target rotation
	 *
	 * @return	True if the tower is at its target rotation
	 */
	bool towerAtPosition();

	/**
	 * Sets all tower motors values so that they move towards
	 * their target positions
	 */
	void update();
	
	// Contains the data needed to control the arm motors. These should
	// be public so they can be initialized in the main loop
	Servo motor_left;
	Servo motor_right;
	Servo arm;
	Servo claw;
	Servo tower;

	// Contains the data needed to read from the arm encoders.  These
	// should be public so they can be initialized in the main loop.
	I2CEncoder encoder_left;
	I2CEncoder encoder_right;
	I2CEncoder encoder_arm;
	I2CEncoder encoder_claw;
	I2CEncoder encoder_tower;
	
	bool hasBottle;			// Whether or not the arm has the bottle

private:
	int targetElevator;		// The target height of the elevator, as a percent
	int targetArm;			// The target extension of the arm, as a percent
	int targetClaw;			// The target opening of the claw, as a percent
	int targetTower;		// The target rotation of the tower, in degrees
};

#endif
