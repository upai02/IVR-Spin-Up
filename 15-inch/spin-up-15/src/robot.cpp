#include "robot.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_mtr(10, false);
pros::Motor right_front_mtr(21, true);
pros::Motor left_back_mtr(19, false);
pros::Motor right_back_mtr(20, true);
pros::Motor_Group left_side({left_front_mtr, left_back_mtr});
pros::Motor_Group right_side({right_front_mtr, right_back_mtr});
pros::Imu imu(3);
