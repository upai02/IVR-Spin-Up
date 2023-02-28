#pragma once
#include "api.h"
#include "robot.h"

extern char auton_sel;

void drivePID(double inches);
void drivePIDodom(double meters);
double getAngleError(double target, double currHeading);
void turnPID(double deg);
void shootPF(double rpm);
void auton_thread();
void auton();

extern pros::Task auton_task;