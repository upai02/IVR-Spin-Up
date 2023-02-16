#include "robot.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
// Drive
pros::Motor left_front_mtr(11, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_top_mtr(15, pros::E_MOTOR_GEARSET_06, false);
pros::Motor left_back_bot_mtr(13, pros::E_MOTOR_GEARSET_06, true);

pros::Motor right_front_mtr(19, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_top_mtr(18, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_back_bot_mtr(20, pros::E_MOTOR_GEARSET_06, false);

pros::Motor_Group left_side({left_front_mtr, left_back_top_mtr, left_back_bot_mtr});
pros::Motor_Group right_side({right_front_mtr, right_back_top_mtr, right_back_bot_mtr});

// Intake
pros::Motor intake_mtr(17, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rai_mtr(16, pros::E_MOTOR_GEARSET_06, true); // indexer + roller mech 
pros::ADIDigitalOut mag_piston(1, true); // mag piston
pros::ADIDigitalOut intake_piston(2, false); // intake piston
// Shooter
pros::Motor flywheel_mtr(14, pros::E_MOTOR_GEARSET_06, true);
pros::ADIDigitalOut angle_piston(3, false); // angle piston

// Sensors
pros::Imu imu(4);
// pros::Optical disc_opt(5);
// pros::Optical roller_opt(6);
pros::ADIEncoder vertical_track(1, 2, false); // tracking wheel #1
pros::ADIEncoder horizontal_track(3, 4, false); // tracking wheel #2
