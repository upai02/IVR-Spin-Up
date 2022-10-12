#include "robot.h"
#include "pros/motors.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_mtr(10, pros::E_MOTOR_GEARSET_18, false);
pros::Motor right_front_mtr(21, pros::E_MOTOR_GEARSET_18,  true);
pros::Motor left_back_mtr(19, pros::E_MOTOR_GEARSET_18, false);
pros::Motor right_back_mtr(20, pros::E_MOTOR_GEARSET_18, true);
pros::Motor_Group left_side({left_front_mtr, left_back_mtr});
pros::Motor_Group right_side({right_front_mtr, right_back_mtr});
pros::Imu imu(3);
pros::Rotation transverseEncoder(18);
pros::ADIEncoder radialEncoder(1, 2, true);