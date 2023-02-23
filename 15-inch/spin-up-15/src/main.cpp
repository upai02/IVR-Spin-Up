#include "main.h"
#include "auton.h"
#include "misc/PositionTracker.h"
#include "intake.h"
#include "endgame.h"
#include "shooter.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	horizontal_track.reset();
	vertical_track.reset();
	// imu.reset(true);
	left_front_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_back_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_back_bot_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_back_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_back_bot_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	flywheel_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rai_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	set_mag_piston(false);
	set_intake_piston(true);
	endgame_released = false;
	flywheel_task.suspend();
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
	// initTracker(0, 0);
	// SmartStop();
	auton();
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
	controls();
}
