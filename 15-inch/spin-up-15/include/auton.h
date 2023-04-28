#pragma once
#include "api.h"
#include "robot.h"
#include "shooter.h"

extern char auton_sel;

void drivePID(double inches);
double getAngleError(double target, double currHeading);
void turnPID(double deg, double kp, double ki, double kd, double max_speed, double min_speed);
void shootPF(double rpm, const double kP = 15.0, const double kF = 24.7);
void auton_thread();
void test_auton();
void auton();
void skill_auton();
void rollerAutoPATH();
void compAutonLeftRobot();
void compAutonRightRobot();
void SAFEcompLeftAuton();
void SAFEcompRightAuton();

void turnP(double deg, double kp, double min_speed, double max_speed);
void assign_min_speed(double &speed, double min_speed);
void assign_max_speed(double &speed, double max_speed);
