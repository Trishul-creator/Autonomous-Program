#ifndef PIDFUNCTIONS_H
#define PIDFUNCTIONS_H

#include "PIDController.h"
#include "vex.h"
#include "RobotConfig.h"

using namespace vex;

void driveToDistance(PIDController& pid, int distance, vex::directionType direction);
void turnToAngle(PIDController& pid, int angle, vex::turnType rightOrLeft);

#endif // PIDFUNCTIONS_H