#include "robot.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_top_mtr(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor left_front_bottom_mtr(10, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_mtr(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_front_top_mtr(2, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_front_bottom_mtr(1, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_mtr(7, pros::E_MOTOR_GEARSET_06, false);

pros::Motor_Group left_side({left_front_top_mtr, left_front_bottom_mtr, left_back_mtr});
pros::Motor_Group right_side({right_front_top_mtr, right_front_bottom_mtr, right_back_mtr});
pros::Motor catapult(19, pros::E_MOTOR_GEARSET_36, false);
pros::Motor intake(6, pros::E_MOTOR_GEARSET_06, true);
pros::Imu imu(20);
pros::ADIEncoder transverseEncoder(1, 2, true);
pros::ADIEncoder radialEncoder(3, 4, true);
pros::ADIDigitalIn cata_limit(5);
pros::Motor roller(18, pros::E_MOTOR_GEARSET_36, false);
pros::ADIDigitalOut endgame_release(8);
// pros::ADIPotentiometer catpot(8, pros::E_ADI_POT_V2);