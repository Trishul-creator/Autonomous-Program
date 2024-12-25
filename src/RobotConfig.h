#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include "vex.h"

using namespace vex;

// Declare motors and sensors as extern variables
extern inertial BrainInertial = inertial();
extern motor LeftDriveSmart = motor(PORT4, 1, false);
extern motor RightDriveSmart = motor(PORT3, 1, true);
extern smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, BrainInertial, 200);
extern brain Brain;
extern motor MotorGroup1MotorA = motor(PORT1, true);
extern motor MotorGroup1MotorB = motor(PORT6, false);
extern motor_group MotorGroup1 = motor_group(MotorGroup1MotorA, MotorGroup1MotorB);



#endif // ROBOTCONFIG_H