#include "PIDController.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "vex.h"
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
    this->error = this->desiredValue;
    this->turnError = this ->turnDesiredValue;
    this->prevError = 0;
    this->turnPrevError = 0;
    this->totalError = 0;
    this->totalTurnError = 0;
    this->derivative = 0;
    this->turnDerivative = 0;
    this->driveErrors = {};
    this->turnErrors = {};
    this->driveOutputs = {};
    this->turnOutputs = {};
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
    this->driveErrors.push_back(this->error);


    
    if (abs(this->error) > this->integralBound) {
        this->totalError += this->error;
    } else {
        totalError = 0;
    }

    int output = (this->kP * this->error) + (this->kI * this->totalError) + (this->kD * this->derivative);

    
    driveOutputs.push_back(output);

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
    this->turnErrors.push_back(this->error);
    
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
    turnOutputs.push_back(output);
    return output;
    
}

bool PIDController::atTarget() {
    return (abs(this->error) < 0.5 && abs(this->error) > -0.5);
}

bool PIDController::atTurnTarget() {
    return (abs(this->turnError) < 0.5 && abs(this->turnError) > -0.5);
}

void PIDController::clearDriveErrorsVector() {
    this->driveErrors.clear();
}
void PIDController::clearTurnErrorsVector() {
    this->turnErrors.clear();
}

void PIDController::clearDriveOutputsVector() {
    this->driveOutputs.clear();
}

void PIDController::clearTurnOutputsVector() {
    this->turnOutputs.clear();
}

void PIDController::printDriveErrors() {
    printf("\nDrive Errors: ");
    for(int i = 0; i < this->driveErrors.size(); i++) {
        printf("%d", this->driveErrors[i]);
        if(i < this->driveErrors.size() - 1) {
            printf(", ");
        }
    }
    printf("\n");
}

void PIDController::printTurnErrors() {
    printf("\nTurn Errors: ");
    for(int i = 0; i < this->turnErrors.size(); i++) {
        printf("%d", this->turnErrors[i]);
        if(i < this->turnErrors.size() - 1) {
            printf(", ");
        }
    }
    printf("\n");
}

void PIDController::printDriveOutputs() {
    printf("\nDrive Outputs: ");
    for(int i = 0; i < this->driveOutputs.size(); i++) {
        printf("%d", this->driveOutputs[i]);
        if(i < this->driveOutputs.size() - 1) {
            printf(", ");
        }
    }
    printf("\n");
}

void PIDController::printTurnOutputs() {
    printf("\nTurn Outputs: ");
    for(int i = 0; i < this->turnOutputs.size(); i++) {
        printf("%d", this->turnOutputs[i]);
        if(i < this->turnOutputs.size() - 1) {
            printf(", ");
        }
    }
    printf("\n");
}