#include "RobotConfig.h"
#include "vex.h"

inertial BrainInertial = inertial();
motor LeftDriveSmart = motor(PORT3, 1, false);
motor RightDriveSmart = motor(PORT9, 1, true);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, BrainInertial, 200);
brain Brain;
motor Launcher = motor(PORT7, true);
motor IntakeMotorA = motor(PORT5, true);
motor IntakeMotorB = motor(PORT6, false);
motor_group IntakeMotor = motor_group(IntakeMotorA, IntakeMotorB);
motor BackRoller = motor(PORT1, true);
PIDController pid = PIDController(0.5, 0.0, 0.0, 0.5, 0.0, 0.0);