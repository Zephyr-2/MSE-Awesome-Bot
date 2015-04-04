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
UltrasonicSensor us(3,2);
long time;
int phase = 0;
float minTheta = 0;
float ir_last;

PID pid_wall_follow(5,3,30);

void setup()
{
	Serial.begin(9600);
	Wire.begin();

	drive.motor_left.attach(10);
	drive.motor_right.attach(11);

	drive.encoder_right.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_right.setReversed(true);  // adjust for positive count when moving forward
	drive.encoder_left.init((10.2 * PI)*(1.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	drive.encoder_left.setReversed(false);  // adjust for positive count when moving forward
        
        //calibrate ir sensor
        for(int i = 0; i < 50; i++) {
          ir_last = ir.read();
          us.read();
        }

        Serial.println("done setup");
	time = millis();
}

void loop()
{
  if(!drive.isReady()) {
    us.read();
    ir_last = ir.read();
  }
  else if(us.read() < WALL_DISTANCE + 5) {
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
  delay(constrain(100 + time - millis(), 0, 100));
}
