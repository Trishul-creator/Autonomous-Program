#include "Launcher.h"

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
