#pragma once
#include "api.h"
#include "robot.h"

void drivePID(double inches);
void turnPID(double degrees);
void shootPF(double rpm);
void auton();