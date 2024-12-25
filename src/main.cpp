#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "PIDFunctions.h"
#include "iq_cpp.h"
#include "RobotConfig.h"
#include "PIDController.h"


#include "vex.h"

using namespace vex;




// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS



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


using namespace vex;




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code
  //create PIDController instance with desired parameters
  PIDController pid(0.25, 0.0, 0.0, 0.25, 0.0, 0.0);
  driveToDistance(pid, 1000, forward);
  turnToAngle(pid, 90, right);
  
} 