#ifndef AUTONOMOUSACTIONS_H
#define AUTONOMOUSACTIONS_H

#include "PIDFunctions.h"
#include "vex.h"
#include "RobotConfig.h"
#include "Launcher.h"

using namespace vex;

extern int points; // Number of points total points throughout autonomous
extern int goalNumber; // Number of goals throughout autonomous
extern int oneSwitchPoint; // Point value for flipping switch
extern int oneSwitchIncrement; // The point multiplier based on number of switches


void preAuton();
void pointCalculator();
void initialShoot();
void shootOtherSide();
void shootLoop(int numOfShots);

#endif // AUTONOMOUSACTIONS_H