#pragma once
#include "api.h"
#include "robot.h"

void drivePID(double inches);
void drivePIDodom(double meters);
double getAngleError(double target, double currHeading);
void turnPID(double degrees);
void shootPF(double rpm);
void auton();