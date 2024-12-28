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
motor LauncherMotorA = motor(PORT1, false);
motor LauncherMotorB = motor(PORT6, true);
motor_group Launcher = motor_group(LauncherMotorA, LauncherMotorB);
motor IntakeMotor = motor(PORT2, true);
motor BackRoller = motor(PORT5, true);

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
  // Calibrate the Drivetrain
  calibrateDrivetrain();
  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//                                                                            
//    Module:       main.cpp                                                  
//    Author:       {author}                                                  
//    Created:      {date}                                                    
//    Description:  IQ project                                                
//                                                                            
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;

// PIDController class definition
class PIDController {
    public:
        PIDController(double kP, double kI, double kD, double turnkP, double turnkI, double turnkD);
        void setDesiredValue(int value);
        void setTurnDesiredValue(int value);
        int calculate(int currentPosition);
        int calculateTurn(int currentAngle);
        bool atTarget();
        bool atTurnTarget();
        int getDesiredValue();

    private:
        double kP; // Proportional constant
        double kI; // Integral constant
        double kD; // Derivative constant

        double turnkP; // Turn Proportional constant
        double turnkI; // Turn Integral constant
        double turnkD; // Turn Derivative constant

        int integralBound; // max integral value
        int maxIntegral; // max integral value

        double desiredValue; // how far it should go
        double turnDesiredValue; // how far it should turn
        double error; // Error = desiredValue - currentPosition (Proportional Term of PID)
        double turnError; // Error = desiredValue - currentPosition (Proportional Term of PID)
        double prevError; // Error from the last time calculate was called
        double turnPrevError; // Error from the last time calculate was called
        double totalError; // integral of error (Integral Term of PID)
        double totalTurnError; // integral of error (Integral Term of PID)
        double derivative; // derivative of error (Derivative Term of PID) error - prevError
        double turnDerivative; // derivative of error (Derivative Term of PID) error - prevError
};

// PIDController class implementation
PIDController::PIDController(double kP, double kI, double kD, double turnkP, double turnkI, double turnkD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    this->turnkP = turnkP;
    this->turnkI = turnkI;
    this->turnkD = turnkD;

    this->integralBound = 3;
    this->maxIntegral = 300;

    this->desiredValue = 0;
    this->turnDesiredValue = 0;
    this->error = 0;
    this->turnError = 0;
    this->prevError = 0;
    this->turnPrevError = 0;
    this->totalError = 0;
    this->totalTurnError = 0;
    this->derivative = 0;
    this->turnDerivative = 0;
}

void PIDController::setDesiredValue(int value) {
    this->desiredValue = value;
}

void PIDController::setTurnDesiredValue(int value) {
    this->turnDesiredValue = value;
}

int PIDController::calculate(int currentPosition) {
    this->error = this->desiredValue - currentPosition;
    printf("Error: %d", this->error);
    this->derivative = this->error - this->prevError;
    
    if (abs(this->error) > this->integralBound) {
        this->totalError += this->error;
    } else {
        totalError = 0;
    }

    int output = (this->kP * this->error) + (this->kI * this->totalError) + (this->kD * this->derivative);

    if(output > 100) {
      output = 100;
    } else if(output < 0) {
      output = 0; 
    }
    this->prevError = this->error;
    return output;
}

int PIDController::getDesiredValue() {
    return this->desiredValue;
}


int PIDController::calculateTurn(int currentAngle) {
    this->turnError = this->turnDesiredValue - currentAngle;
    this->turnDerivative = this->turnError - this->turnPrevError;
    
    if (abs(this->turnError) > this->integralBound) {
        this->totalTurnError += this->turnError;
    } else {
        totalTurnError = 0;
    }

    int output = (this->turnkP * this->turnError) + (this->turnkI * this->totalTurnError) + (this->turnkD * this->turnDerivative);

    if(output > 100) {
      output = 100;
    } else if(output < 0) {
      output = 0; 
    }
    this->turnPrevError = this->turnError;
    return output;
}

bool PIDController::atTarget() {
    return (abs(this->error) < 0.5 && abs(this->error) > -0.5);
}

bool PIDController::atTurnTarget() {
    return (abs(this->turnError) < 0.5 && abs(this->turnError) > -0.5);
}

// Drive function using PID
void drive(PIDController& pid, int distance, vex::directionType direction) {
  pid.setDesiredValue(distance);
  printf("Desired Value: %d", pid.getDesiredValue());
  

  while (!pid.atTarget()) {
    int leftMotorPosition = LeftDriveSmart.position(degrees) * 200 * M_PI / 360;
    int rightMotorPosition = RightDriveSmart.position(degrees) * 200 * M_PI / 360;
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;

    int motorPower = pid.calculate(averagePosition);
     printf("Motor Power: %d", motorPower);

    LeftDriveSmart.spin(direction, motorPower, percent);
    RightDriveSmart.spin(direction, motorPower, percent);
    wait(20, msec);
  }
  Drivetrain.stop();
}

// Turn function using PID
void turn(PIDController& pid, int angle, vex::turnType rightOrLeft) {
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

// Launcher functions
void shootTopGoal() {
    Launcher.spinFor(forward, 40, degrees);
}

void setPositionForShooting() {
    Launcher.spinFor(reverse, 615, degrees, false);
}

void backRollerLauncherPosition() {
    Launcher.spinFor(forward, 430, degrees, false);
}

void backRollerLauncherPositionRelease() {
    Launcher.spinFor(reverse, 20, degrees);
}

// PIDController instance
PIDController pid = PIDController(0.5,0.0,0.0, 0.5, 0.0, 0.0);

// Autonomous variables
int points = 0;
int goalNumber = 0;
int oneSwitchPoint = 0;
int oneSwitchIncrement = 0;

// Point calculator function
void pointCalculator(bool switchIncrementDesicision) { // switchIncrementDesicion, if switch cleared or not
    Brain.playSound(tada); // Play sound for getting goal
    if(switchIncrementDesicision == true) {
        oneSwitchIncrement += 4; // Increment the point value for each switch cleared
        printf("1 switch cleared !"); // Print to the console that a switch was cleared
        printf("Every goal is worth %d points", oneSwitchIncrement); // Print to the console the new point value
    }
        goalNumber += 1; // Increment the goal number
        points = (goalNumber*oneSwitchIncrement)+(oneSwitchPoint*(oneSwitchIncrement/4)); // Calculate the total points
        printf("%dst Goal Shot !", goalNumber); // Print to the console the goal number
        printf("The score is %d", points); // Print to the conosole the total points
}

// Pre-autonomous function
void preAuton() {
    // Set velocity and turn parameters
    Launcher.setVelocity(100, percent); // Set the velocity of the launcher to 100 percent
    printf("Launcher velocity set to 100 percent"); // Print to the console the velocity of the launcher
    IntakeMotor.setVelocity(100, percent); // Set the velocity of the intake motor to 100 percent
    printf("Intake velocity set to 100 percent"); // Print to the console the velocity of the intake motor
    BackRoller.setVelocity(100, percent); // Set the velocity of the back roller to 100 percent
    printf("Back roller velocity set to 100 percent"); // Print to the console the velocity of the back roller
    setPositionForShooting();
}

// Initial shoot function
void initialShoot() {
    Drivetrain.setHeading(0, degrees); // Set the heading of the drivetrain to 0 degrees
    drive(pid, 400, reverse); // Drive the robot forward 400 degrees (with PID) toward the ball
    IntakeMotor.spinFor(forward, 2.5, turns); // Spin the intake motor forward for 2.5 turns to collect ball
    turn(pid, 180, right); // Turn the robot 180 degrees to the right (with PID) with both balls
    drive(pid, 550, forward); // Drive the robot forward 550 degrees (with PID) with both balls
    BackRoller.spinFor(forward, 0.3, turns); // Spin the back roller forward for 0.3 turns to shoot the preloaded ball into bottom goal
    pointCalculator(true); // Increment the goal number and calculate the total points
    shootTopGoal(); // Shoot the top goal
    pointCalculator(true); // Increment the goal number and calculate the total points
}

// Shoot other side function
void shootOtherSide() {
    setPositionForShooting(); // Set the position for shooting
    turn(pid, 30, right); 
    drive(pid, 1097, reverse);
    IntakeMotor.spinFor(forward, 2.5, turns);
    turn(pid, 30, left);
    drive(pid, 950, forward);
    shootTopGoal();
    pointCalculator(true);
    IntakeMotor.spinFor(forward, 2.5, turns, false);
    BackRoller.spinFor(forward, 2.5, turns);
    pointCalculator(true);
}

// Shoot loop function
void shootLoop(int numOfShots) {
    for(int i = 0; i < numOfShots; i++) {
        backRollerLauncherPosition();
        drive(pid, 920, reverse);
        IntakeMotor.spinFor(forward, 2.5, turns);
        drive(pid, 950, forward);
        shootTopGoal();
        pointCalculator(true);
        IntakeMotor.spinFor(forward, 2.5, turns, false);
        BackRoller.spinFor(forward, 2.5, turns);
        pointCalculator(true);
        wait(100, msec);
    }
}

// Autonomous actions function
void autonomousActions() {
  preAuton();
  initialShoot();
  shootOtherSide();
  shootLoop(10);
}

// Autonomous function
void autonomous() {
  preAuton();
  Brain.buttonUp.pressed(autonomousActions);
  Brain.Screen.print("Autonomous Started");
}

// Autonomous timer function
void autonomousTimer() {
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

// Main function
int main() {
  vexcodeInit();
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