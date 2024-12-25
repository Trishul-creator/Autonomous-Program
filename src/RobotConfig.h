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
extern motor LauncherMotorA;
extern motor LauncherMotorB;
extern motor_group Launcher;
extern motor IntakeMotor;
extern motor BackRoller;
extern PIDController pid;


#endif // ROBOTCONFIG_H