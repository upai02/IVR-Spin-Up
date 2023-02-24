#pragma once
#include "api.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"

extern pros::Controller master;
extern pros::Motor left_front_top_mtr;
extern pros::Motor right_front_top_mtr;
extern pros::Motor left_back_mtr;
extern pros::Motor right_back_mtr;
extern pros::Motor left_front_bottom_mtr;
extern pros::Motor right_front_bottom_mtr;
extern pros::Motor_Group left_side;
extern pros::Motor_Group right_side;
extern pros::Motor catapult;
extern pros::Motor intake;
extern pros::Imu imu;
extern pros::ADIEncoder transverseEncoder;
extern pros::ADIEncoder radialEncoder;
extern pros::ADIDigitalIn cata_limit;
extern pros::ADIPotentiometer catpot;
extern pros::Motor roller;
