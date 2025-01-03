#include "PIDFunctions.h"

void drive(PIDController& pid, int distance, vex::directionType direction) {

  const double WHEEL_DIAMETER = 200.0;
  const double MM_TO_DEG = 360.0 / (WHEEL_DIAMETER * M_PI);
  int targetDegrees = distance * MM_TO_DEG;
  pid.setDesiredValue(targetDegrees);
  printf("Desired Value: %d", pid.getDesiredValue());
  double InitialHeading = BrainInertial.heading();

  while (true) {
    int leftMotorPosition = LeftDriveSmart.position(degrees);
    int rightMotorPosition = RightDriveSmart.position(degrees);
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;

    int motorPower = pid.calculate(averagePosition);

    
    double headingError = InitialHeading - BrainInertial.heading();
    double turnPower = headingError * 0.5;
      
   
    LeftDriveSmart.spin(direction, motorPower - turnPower , percent);
    RightDriveSmart.spin(direction, motorPower + turnPower, percent);
    if(abs(LeftDriveSmart.velocity(percent)) < 5 && abs(RightDriveSmart.velocity(percent)) < 5) {
      break;
    }
    wait(20, msec);
  }
  pid.printDriveErrors();
  pid.printDriveOutputs();
  pid.clearDriveErrorsVector();
  pid.clearDriveOutputsVector();
  
}

void turn(PIDController& pid, int angle, vex::turnType rightOrLeft) {
  pid.setTurnDesiredValue(angle);

  while (true) {
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
    if(abs(LeftDriveSmart.velocity(percent)) < 5 && abs(RightDriveSmart.velocity(percent)) < 5) {
      break;
    }
    wait(20, msec);
  }
  pid.printTurnErrors();
  pid.printTurnOutputs();
  pid.clearTurnErrorsVector();
  pid.clearTurnOutputsVector();
}
