#include "robot.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_mtr(8, true);
pros::Motor right_front_mtr(5, false);
pros::Motor left_back_mtr(3, true);
pros::Motor right_back_mtr(7, false);
pros::Motor_Group left_side({left_front_mtr, left_back_mtr});
pros::Motor_Group right_side({right_front_mtr, right_back_mtr});
pros::Imu imu(4);
