#include "RobotConfig.h"
#include "vex.h"

inertial BrainInertial = inertial();
motor LeftDriveSmart = motor(PORT4, 1, false);
motor RightDriveSmart = motor(PORT3, 1, true);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, BrainInertial, 200);
brain Brain;
motor MotorGroup1MotorA = motor(PORT1, true);
motor MotorGroup1MotorB = motor(PORT6, false);
motor_group MotorGroup1 = motor_group(MotorGroup1MotorA, MotorGroup1MotorB);