#pragma once
#include "api.h"
#include "pros/motors.h"
#include "robot.h"

double normalize_joysticks(double input);
double sin_scale(double input);
double square_scale(double input);
void tank_drive();
void arcade_drive();
void hybrid_drive();
void controls();
void group_stop(pros::motor_brake_mode_e b);
void move_with_assigned_speed (double xVel, double yVel, double turnVel);
void wannabeSwerve();