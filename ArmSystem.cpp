#include "ArmSystem.h"

ArmSystem::ArmSystem() : 
pid_elevator(1,0,0)
{
  targetElevator = 0;
  targetArm = 0;
  targetClaw = 0;

  encoder_left.zero();
  encoder_right.zero();
  encoder_arm.zero();
  encoder_claw.zero();
}

void ArmSystem::setElevator(int targetElevator)
{
  this->targetElevator = targetElevator;
  pid_elevator.reset();
}

void ArmSystem::setArm(int targetArm)
{
  this->targetArm = targetArm;
}

void ArmSystem::setClaw(int targetClaw)
{
  this->targetClaw = targetClaw;
}

bool ArmSystem::elevatorAtPosition()
{
  if(abs(targetElevator - encoder_left.getPosition()) < ELEVATOR_THRESHOLD)
    return true;
  return false;
}

bool ArmSystem::armAtPosition()
{
  if(abs(targetArm - encoder_arm.getPosition()) < ARM_THRESHOLD)
    return true;
  return false;
}

bool ArmSystem::clawAtPosition()
{
  if(abs(targetClaw - encoder_claw.getPosition()) < CLAW_THRESHOLD)
    return true;
  return false;
}

void ArmSystem::update()
{
  float ele = targetElevator - encoder_left.getPosition();
  if(abs(ele) > THRESHOLD) {
    motor_left.writeMicroseconds(MOTOR_BRAKE + (int)ele * 200);
    motor_right.writeMicroseconds(MOTOR_BRAKE + (int)ele * 200);
  }
  else {
    motor_left.writeMicroseconds(MOTOR_BRAKE);
    motor_right.writeMicroseconds(MOTOR_BRAKE);
  }

  float arm1 = targetArm - encoder_arm.getPosition();
  if(abs(arm1) > THRESHOLD)
  {
    arm.writeMicroseconds(MOTOR_BRAKE + (int)arm1 * 200);
  }
  else
  {
   arm.writeMicroseconds(MOTOR_BRAKE);
  }

  float claw1 = targetClaw - encoder_claw.getPosition();
  if(abs(claw1) > THRESHOLD)
  {
    claw.writeMicroseconds(MOTOR_BRAKE + (int)claw1 * 200);
  }
  else
  {
    claw.writeMicroseconds(MOTOR_BRAKE);
  }
}

