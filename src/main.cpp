#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "PIDFunctions.h"
#include "iq_cpp.h"
#include "vex.h"
#include "RobotConfig.h"
#include "PIDController.h"
#include "AutonomousActions.h"

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


void autonomousActions() {
  //queue all autonomous actions here
  preAuton();
  pointCalculator();
  initialShoot();
  shootOtherSide();
  shootLoop(10);
}

void autonomous() {
  // Call preAuton and Autonomous Actions
  preAuton();
  Brain.buttonUp.pressed(autonomousActions);
  Brain.Screen.print("Autonomous Started");
}

void autonomousTimer() {
  // Timer for autonomous
  int time = 60;

  while(time != 0) {
    wait(1, seconds);
    time--;
    if(time == 10 || time == 20 || time == 30 || time == 40 || time == 50) {
      printf("The time left is %d",time);
    }
  }
  printf("Autonomous Ended");
  Brain.playSound(powerDown);
  Brain.Screen.print("Autonomous Ended");
  Brain.programStop();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code
  //create PIDController instance with desired parameters
  bool test = true;

  if(test == true) {
    drive(pid, 1000, forward);
    turn(pid, 90, right);
  } else {
    thread timerThread = thread(autonomousTimer);
    thread autonomousThread = thread(autonomous);
    timerThread.detach();
    autonomousThread.detach();
  }
  
} 