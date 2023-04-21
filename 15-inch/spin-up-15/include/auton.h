#pragma once
#include "api.h"
#include "robot.h"

extern char auton_sel;

void drivePID(double inches);
double getAngleError(double target, double currHeading);
void turnPID(double deg, double kp, double ki, double kd, double max_speed, double min_speed);
void shootPF(double rpm);
void auton_thread();
void test_auton();
void auton();
void skill_auton();
void rollerAutoPATH();
void compAutonLeftRobot();
void compAutonRightRobot();

extern pros::Task auton_task;