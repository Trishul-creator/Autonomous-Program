#include "AutonomousActions.h"

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

// Shoot the goal forward
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

void shootOtherSide() {
    // Shoot other side goals
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

void shootLoop(int numOfShots) {
    // Shoot Loop
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
