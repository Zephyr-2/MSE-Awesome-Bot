#include "ArmSystem.h"

ArmSystem::ArmSystem()
{
  targetElevator = 0;
  targetArm = 0;
  targetClaw = 0;
  targetTower = 0;
}

void ArmSystem::setElevator(int targetElevator)
{
  this->targetElevator = targetElevator;
}

void ArmSystem::setArm(int targetArm)
{
  this->targetArm = targetArm;
}

void ArmSystem::setClaw(int targetClaw)
{
  this->targetClaw = targetClaw;
}

void ArmSystem::setTower(int targetTower)
{
  this->targetTower = targetTower;
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
  float tmp = targetElevator - encoder_left.getPosition();
  if(tmp > ELEVATOR_THRESHOLD) {
    motor_left.writeMicroseconds(ELEVATOR_UNLOAD_UP);
    motor_right.writeMicroseconds(ELEVATOR_UNLOAD_UP);
  }
  else if(tmp < -ELEVATOR_THRESHOLD) {
    motor_left.writeMicroseconds(ELEVATOR_UNLOAD_DOWN);
    motor_right.writeMicroseconds(ELEVATOR_UNLOAD_DOWN);
  }
  else if(targetElevator == 0) {
    motor_left.writeMicroseconds(MOTOR_BRAKE);
    motor_right.writeMicroseconds(MOTOR_BRAKE);
  }
  else {
    motor_left.writeMicroseconds(ELEVATOR_UNLOAD_BRAKE);
    motor_right.writeMicroseconds(ELEVATOR_UNLOAD_BRAKE);
  }

  tmp = targetArm - encoder_arm.getPosition();
  if(tmp > ARM_THRESHOLD) {
    arm.writeMicroseconds(ARM_FOREWARD);
  }
  else if(tmp < -ARM_THRESHOLD) {
    arm.writeMicroseconds(ARM_RETRACT);
  }
  else {
   arm.writeMicroseconds(MOTOR_BRAKE);
  }

  tmp = targetClaw - encoder_claw.getPosition();
  if(tmp > CLAW_THRESHOLD) {
    claw.writeMicroseconds(CLAW_OPEN);
  }
  else if(tmp < -CLAW_THRESHOLD) {
    claw.writeMicroseconds(CLAW_CLOSE);
  }
  else {
    claw.writeMicroseconds(MOTOR_BRAKE);
  }
  
  tmp = targetTower - encoder_tower.getPosition();
  if(tmp > TOWER_THRESHOLD) {
    tower.writeMicroseconds(TOWER_LEFT);
  }
  else if(tmp < -TOWER_THRESHOLD) {
    tower.writeMicroseconds(TOWER_RIGHT);
  }
  else {
    tower.writeMicroseconds(MOTOR_BRAKE);
  }
}

