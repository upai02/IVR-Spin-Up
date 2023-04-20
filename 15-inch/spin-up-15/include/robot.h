#pragma once
#include "api.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"

extern pros::Controller master;
// Drivetrain 
extern pros::Motor left_front_mtr;
extern pros::Motor left_back_bot_mtr;
extern pros::Motor left_back_top_mtr;
extern pros::Motor right_front_mtr;
extern pros::Motor right_back_bot_mtr;
extern pros::Motor right_back_top_mtr;
extern pros::Motor_Group left_side;
extern pros::Motor_Group right_side;
// Intake 
extern pros::Motor intake_mtr;
extern pros::Motor rai_mtr;
extern pros::ADIDigitalOut mag_piston;
// Shooter 
extern pros::Motor flywheel_left_mtr;
extern pros::Motor flywheel_right_mtr;
extern pros::Motor_Group flywheel;
// Endgame
extern pros::ADIDigitalOut eg_deploy_piston; // deploys endgame/blocker
extern pros::ADIDigitalOut string_release_piston; // releases string
// Sensors
extern pros::Imu imu;
extern pros::Distance disc_dist;
extern pros::Optical roller_opt;
extern pros::ADIEncoder vertical_track; // tracking wheel #1
extern pros::ADIEncoder horizontal_track; // tracking wheel #2
