#pragma once
#include "api.h"
#include "robot.h"
#include "shooter.h"

extern char auton_sel;

void drivePID(double inches);
double getAngleError(double target, double currHeading);
void turnPID(double deg, double kp, double ki, double kd, double max_speed, double min_speed);
void shootPF(double rpm);
void auton_thread();
void auton();

void turnP(double deg, double kp, double min_speed, double max_speed);
void assign_min_speed(double &speed, double min_speed);
void assign_max_speed(double &speed, double max_speed);
