#include "main.h"
#include "auton.h"
#include <vector>
#include "controls.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "misc/PositionTracker.h"

/* temp import */
#include "movement.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset(true);
	left_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	transverseEncoder.reset();
	radialEncoder.reset();

	// ROLLER AUTO
	std::vector<double> starting_position = {0.9, 0.41}; // 35in, 16in
	// // Edges to walls: 7.5in Y, 29in X
	initTracker(starting_position[0], starting_position[1]);


	// // SKILLS
	// std::vector<std::vector<double>> skillsPathSeg1 = {{1.42, 3.4}, {1.8, 3.4}}; // reversed, facing 270
	// // starting x is with front of robot on opponent low goal plane, y is against wall.
	// initTracker(skillsPathSeg1[0][0], skillsPathSeg1[0][1]);

	// ENDGAME SKILLS AUTO
    // std::vector<double> start_position = {0.89, 0.4}; // 7 inches (0.18 meters) off wall
    // - back of robot touching vertical plane created by furthest edge of the 2nd foam tile into the field
	// initTracker(start_position[0], start_position[1]);


	// STRAIGHT PATH TEST
	// initTracker();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::Task position_updater(update_position);
	SmartStop();
	// ROLLER AUTO
	// rollerAuto();

	// SKILLS AUTO
	// boltEndgameAuto();
	// boltSkillsAuto();

	// STRAIGHT PATH TEST
	// StraightPathTest();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// autonomous();

	// imu.reset();
	// pros::delay(5000);
	pros::Task position_updater(update_position);
	controls();
}
