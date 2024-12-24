#include "PIDController.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "math.h"



PIDController::PIDController(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralBound = 3;
    this->maxIntegral = 300;
    this->desiredValue = 0;
    this->error = 0;
    this->prevError = 0;
    this->totalError = 0;
    this->derivative = 0;
}

void PIDController::setDesiredValue(int value) {
    this->desiredValue = value;
}

int PIDController::signnum_c(int x) {
    if (x > 0) {
        return 1;
    } else if (x < 0) {
        return -1;
    } else {
        return 0;
    }
}

int PIDController::calculate(int currentPosition) {
    this->error = this->desiredValue - currentPosition;
    this->derivative = this->error - this->prevError;
    this->totalError += this->error;
    
    if (abs(this->error) > this->integralBound) {
        this->totalError += this->error;
    } else {
        totalError = 0;
    }
    totalError = abs(totalError) > maxIntegral ? (totalError > 0 ? maxIntegral : -maxIntegral) : totalError;

    int output = (this->kP * this->error) + (this->kI * this->totalError) + (this->kD * this->derivative);
    this->prevError = this->error;
    return output;
    
}

bool PIDController::atTarget() {
    return abs(this->error) < 1;
}