#pragma once
#include "api.h"
#include "pros/motors.h"
#include "robot.h"

// void tank();
// void arcade();
void mecanum();
void controls();
void group_stop(pros::motor_brake_mode_e b);
void move_with_assigned_speed (double xVel, double yVel, double turnVel);
void wannabeSwerve();
