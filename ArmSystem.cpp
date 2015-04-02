#include "ArmSystem.h"

ArmSystem::ArmSystem() : pid_elevator(1,0,0)
{
  targetelevation = 0;
  targetlength = 0;
  targetclawopen = 0;

  encoder_left_elevator.zero();
  encoder_right_elevator.zero();
  armposition = encoder_arm.zero();
  clawposition = enoder_claw.zero();
}

void ArmSystem::moveElevator(int targetElevation)
{
  this->targetElevation = targetElevation;
  pid_elevator.reset();
  /*float elevatorpositionleftnew = elevatorpositionleft + targetelevation;
  float elevatorpositionleftnew = elevatorpositionright + targetelevation;

  if (targetelevation > 0)
  {
    while( (encoder_left_elevator.getposition() < elevatorpositionleftnew) && (encoder_right_elevator.getposition() < elevatorpositionrightnew) )
    {
      left_elevator.writeMicroseconds(1700);
      right_elevator.writeMicroseconds(1700);
    }
  }
  else
  {
    while( (encoder_left_elevator.getposition() > elevatorpositionleftnew) && (encoder_right_elevator.getposition() > elevatorpositionrightnew) )
    {
      left_elevator.writeMicroseconds(1300);
      right_elevator.writeMicroseconds(1300);
    }
  }*/
}

void update()
{
  int elevator_speed = MOTOR_BRAKE + pid_elevator.update(targetElevation - encoder_left_elevator.getPosition());
  left_elevator.writeMicroseconds(elevator_speed);
  right_elevator.writeMicroseconds(elevator_speed);
}

bool elevatorAtPosition()
{
  if(abs(targetElevation - encoder_left_elevator.getPosition()) < THRESHOLD)
    return true;
  return false;
}

void ArmSystem::extendarm(int targetlength)
{
  this->targetlength = targetlength;

  float newarmposition = armposition + targetlength;

  if (targetlength > 0)
  {
    while( (encoder_arm.getposition() < newarmposition) )
    {
      arm.writeMicroseconds(1700); 
    }
  }
  else
  {
    while( (encoder_arm.getposition() > newarmPosition) )
    {
      arm.writeMicroseconds(1300); 
    }
  }
}

void ArmSystem::openclaw(int targetclawopen)
{
  this->targetclawopen = targetclawopen;

  float newclawposition = clawposition + targetclawopen;

  if (targetclawopen > 0)
  {
    while( (encoder_claw.getposition() < newclawposition) )
    {
      claw.writeMicroseconds(1700);
    } 
  }
  else
  {
    while( (encoder_claw.getposition() > newclawposition) )
    {
      claw.writeMicroseconds(1300);
    } 
  }
}


