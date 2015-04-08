/**
 * Controls the drive motors of the robot. Also keeps a running
 * calculation of the robot's current position (in cm) and
 * angle (in radians) from its starting location
 *
 * Author: Robert Meagher
 * Written for MSE2202B 2015
 */ 

#ifndef DRIVESYSTEM_H
#define DRIVESYSTEM_H

#include "Arduino.h"
#include "PID.h"
#include "Point.h"
#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

// List of possible drive states
#define DRIVE 			0
#define TURN 			1
#define STOP 			2
#define TURN 			3
#define TURN_ANGLE 		4

#define MOTOR_MAX 		2100
#define MOTOR_MIN 		800
#define MOTOR_BRAKE 	1500

// The width between the drive wheels, in centimetres
#define CAL_WIDTH 		21

// The allowable tolerance from a turn, in radians
#define ANGLE_THRESHOLD 0.1

 class DriveSystem
 {
 public:
 	DriveSystem();

 	/**
 	 * Sets the robot to drive straight at a given speed
 	 *
 	 * @param	targetSpeed 	The speed to set the robot to
 	 */
 	void drive(int targetSpeed);

 	/**
 	 * Stops the robot
 	 */
 	void brake();

 	/**
 	 * Sets the robot to drive straight at a given speed
 	 *
 	 * @param	targetSpeed 	The speed to set the robot to
 	 * @param	turnRadius		The angle to turn the robot to. Must
 	 *							be between -1000 and 1000.
 	 *							-1000: Turns left on the spot
 	 *							 -500: Turns only right wheel
 	 *								0: Drives straight
 	 *							  500: Turns only left wheel
 	 *							 1000: Turns right on the spot
 	 */
 	void turn(int targetSpeed, int turnRadius);

 	/**
 	 * Turns the robot to a designated angle
 	 *
 	 * @param	targetSpeed		The speed to drive the motors
 	 * @param	targetTheta		The angle to set the robot to
 	 * @param	isRelative		Whether set angle is measured
 	 *							from the robot's current position
 	 *							(true) or from its start position
 	 *							(false)
 	 */
 	void turnToAngle(int targetSpeed, float targetTheta, bool isRelative);

 	/**
 	 * Returns true if the robot isn't turning.
 	 *
 	 * @return 	Whether the robot is ready to be controlled
 	 */
 	bool isReady();

 	/**
 	 * Updates the current position and state of the robot,
 	 * and sets the motors to whatever state the robot was
 	 * set to.
 	 */
 	void update();

 	float getTheta()    { return currentTheta; }
 	Point getPosition() { return currentPosition; }

 	// Servo objects to control the motors
 	Servo motor_left;
 	Servo motor_right;

 	// Encoder objects to read from the motor encoders
 	I2CEncoder encoder_left;
 	I2CEncoder encoder_right;

 	// The last time the drive system was updated
 	int lastTime;

 private:
 	/**
 	 * Updates the current position and angle of the robot
 	 * Must be called every time the robot updates to minimize drift
 	 */
 	void updatePositionAndTheta();

	/**
	 * Constrains an angle between -PI and PI
	 *
	 * @param theta 	The angle to constrain
	 * @return			The angle, constrained between -PI
	 *					radians and PI radians
	 */
	float constrainTheta(float theta);

	Point currentPosition;		// Current position from origin
	float currentTheta;			// Current angle from the beginning

	int targetSpeed;			// The target speed of the robot
	float targetTheta;			// The target angle of the robot

	float encoder_left_last;	// The last encoder reading of the left motor
	float encoder_right_last;	// The last encoder reading of the right motor

	byte state;					//The current state of the robot
};

#endif
