#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include "vex.h"
#include "PIDController.h"

using namespace vex;

// Declare motors and sensors as extern variables
extern inertial BrainInertial;
extern motor LeftDriveSmart;
extern motor RightDriveSmart;
extern smartdrive Drivetrain;
extern brain Brain;
extern motor Launcher;
extern motor IntakeMotorA;
extern motor IntakeMotorB;
extern motor_group IntakeMotor;
extern motor BackRoller;
extern PIDController pid;


#endif // ROBOTCONFIG_H