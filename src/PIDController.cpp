#include "PIDController.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "math.h"



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