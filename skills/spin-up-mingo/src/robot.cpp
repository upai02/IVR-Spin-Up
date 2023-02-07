#include "robot.h"
#include "pros/adi.h"
#include "pros/motors.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_mtr(11, true);
pros::Motor right_front_mtr(16, false);
pros::Motor left_back_top_mtr(4, true);
pros::Motor right_back_top_mtr(6, false);
pros::Motor left_back_bottom_mtr(3, true);
pros::Motor right_back_bottom_mtr(7, false);
pros::Motor_Group left_side({left_front_mtr, left_back_top_mtr, left_back_bottom_mtr});
pros::Motor_Group right_side({right_front_mtr, right_back_top_mtr, right_back_bottom_mtr});
pros::Motor catapult(10, pros::E_MOTOR_GEARSET_36, false);
pros::Motor intake(2, true);
pros::Motor indexer(1, false);
pros::Motor roller(5, false);
pros::Imu imu(3);
pros::Rotation transverseEncoder(18);
pros::ADIEncoder radialEncoder(1, 2, true);
pros::ADIPotentiometer catpot(8, pros::E_ADI_POT_V2);