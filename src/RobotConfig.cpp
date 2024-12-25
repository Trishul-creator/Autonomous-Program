#include "RobotConfig.h"
#include "vex.h"

inertial BrainInertial = inertial();
motor LeftDriveSmart = motor(PORT4, 1, false);
motor RightDriveSmart = motor(PORT3, 1, true);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, BrainInertial, 200);
brain Brain;
motor LauncherMotorA = motor(PORT1, false);
motor LauncherMotorB = motor(PORT6, true);
motor_group Launcher = motor_group(LauncherMotorA, LauncherMotorB);
motor IntakeMotor = motor(PORT2, true);
motor BackRoller = motor(PORT5, true);
PIDController pid = PIDController(0.5, 0.0, 0.0, 0.5, 0.0, 0.0);