#include "robot.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
// Drive
pros::Motor left_front_mtr(11, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_top_mtr(13, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_bot_mtr(12, pros::E_MOTOR_GEARSET_06, false);

pros::Motor right_front_mtr(18, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_top_mtr(20, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_bot_mtr(19, pros::E_MOTOR_GEARSET_06, true);

pros::Motor_Group left_side({left_front_mtr, left_back_top_mtr, left_back_bot_mtr});
pros::Motor_Group right_side({right_front_mtr, right_back_top_mtr, right_back_bot_mtr});

// Intake
pros::Motor intake_mtr(6, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rai_mtr(7, pros::E_MOTOR_GEARSET_06, true); // indexer + roller mech 
pros::ADIDigitalOut mag_piston('h', true); // mag piston
// Shooter
pros::Motor flywheel_left_mtr(14, pros::E_MOTOR_GEARSET_06, true);
pros::Motor flywheel_right_mtr(15, pros::E_MOTOR_GEARSET_06, false);
pros::Motor_Group flywheel({flywheel_left_mtr, flywheel_right_mtr});
// Endgame
pros::ADIDigitalOut eg_deploy_piston('f', false); // blocker deploy piston
pros::ADIDigitalOut string_release_piston('g', false); // string release piston
// Sensors
pros::Imu imu(8);
pros::Distance disc_dist(5);
pros::Optical roller_opt(9);
pros::ADIEncoder vertical_track(3, 4, true); // tracking wheel #1
pros::ADIEncoder horizontal_track(1, 2, true); // tracking wheel #2