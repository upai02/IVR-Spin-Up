#pragma once
#include "api.h"
#include "pros/motors.h"
#include "robot.h"
#include "pros/apix.h" // to include semaphores

/* tasks and semaphores to be defined in controls.cpp*/
extern pros::Task *resetCata_ptr;
extern pros::c::sem_t cata_reset_sem;

void resetCata (void* param);
void singleShot();

double normalize_joysticks(double input);
double sin_scale(double input);
double power_inputs(double input, double power = 2.0);
void tank_drive();
void arcade_drive(bool shooterForward = true);
void hybrid_drive();
void controls();
void group_stop(pros::motor_brake_mode_e b);
void move_with_assigned_speed (double xVel, double yVel, double turnVel);
void wannabeSwerve();
void shootAndWait();
void release_endgame_spools();
void ResetShooterLoop();
void ShootDisksBlocking();
