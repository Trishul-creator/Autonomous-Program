#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor LeftDriveSmart = motor(PORT4, 1, false);
motor RightDriveSmart = motor(PORT3, 1, true);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, BrainInertial, 200);



// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}

bool vexcode_initial_drivetrain_calibration_completed = false;
void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  vexcode_initial_drivetrain_calibration_completed = true;
  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

void vexcodeInit() {
  Drivetrain.setStopping(hold);
  // Calibrate the Drivetrain
  calibrateDrivetrain();

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       gurumurthyvenkataraman                                    */
/*    Created:      12/23/2024, 6:02:10 PM                                    */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the IQ Library
#include "iq_cpp.h"
#include "PIDController.h"


using namespace vex;

void driveToDistance(PIDController& pid, int distance, vex::directionType direction) {
  pid.setDesiredValue(distance);

  while (!pid.atTarget()) {
    int leftMotorPosition = LeftDriveSmart.position(degrees) * 200 * M_PI / 360;
    int rightMotorPosition = RightDriveSmart.position(degrees) * 200 * M_PI / 360;
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;

    int motorPower = pid.calculate(averagePosition);

    

    LeftDriveSmart.spin(direction, motorPower, percent);
    RightDriveSmart.spin(direction, motorPower, percent);
    wait(20, msec);
  }
  Drivetrain.stop();
}

void turnToAngle(PIDController& pid, int angle, vex::turnType rightOrLeft) {
  pid.setTurnDesiredValue(angle);

  while (!pid.atTurnTarget()) {
    int currentAngle = BrainInertial.rotation();
    int turnPower = pid.calculateTurn(currentAngle);

    if(rightOrLeft == right) {
      LeftDriveSmart.spin(forward, turnPower, percent);
      RightDriveSmart.spin(reverse, turnPower, percent);
    } else if(rightOrLeft == left){
      LeftDriveSmart.spin(reverse, turnPower, percent);
      RightDriveSmart.spin(forward, turnPower, percent);
    } else {
      Brain.Screen.print("Invalid turn type");
    }
    wait(20, msec);
  }
  Drivetrain.stop();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code

  //create PIDController instance with desired parameters
  PIDController pid(0.25, 0.0, 0.0, 0.25, 0.0, 0.0);

  driveToDistance(pid, 1000, forward);

  turnToAngle(pid, 90, right);
  
}
