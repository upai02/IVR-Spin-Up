#pragma once
#include "api.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"

extern pros::Controller master;
extern pros::Motor left_front_mtr;
extern pros::Motor right_front_mtr;
extern pros::Motor left_back_top_mtr;
extern pros::Motor right_back_top_mtr;
extern pros::Motor left_back_bottom_mtr;
extern pros::Motor right_back_bottom_mtr;
extern pros::Motor_Group left_side;
extern pros::Motor_Group right_side;
extern pros::Motor catapult;
extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor roller;
extern pros::Imu imu;
extern pros::Rotation transverseEncoder;
extern pros::ADIEncoder radialEncoder;
extern pros::ADIPotentiometer catpot;
