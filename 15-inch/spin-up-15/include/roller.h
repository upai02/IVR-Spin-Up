#pragma once
#include "robot.h"

// made up these values for now
const double BLUE_ROLLER_LOWER_HUE = 200;
const double BLUE_ROLLER_UPPER_HUE = 250;

const double RED_ROLLER_LOWER_HUE = 0;
const double RED_ROLLER_UPPER_HUE = 50;

void spin_roller();
void spin_roller(double rotations_to_spin);