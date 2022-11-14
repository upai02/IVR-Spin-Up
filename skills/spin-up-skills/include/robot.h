#pragma once
#include "api.h"
#include "pros/imu.hpp"

extern pros::Controller master;
extern pros::Motor left_front_mtr;
extern pros::Motor right_front_mtr;
extern pros::Motor left_back_mtr;
extern pros::Motor right_back_mtr;
extern pros::Motor_Group left_side;
extern pros::Motor_Group right_side;
extern pros::Motor left_flywheel;
extern pros::Motor right_flywheel;
extern pros::Motor_Group flywheel;
extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor roller;
extern pros::Imu imu;
extern pros::Rotation transverseEncoder;
extern pros::ADIEncoder radialEncoder;
