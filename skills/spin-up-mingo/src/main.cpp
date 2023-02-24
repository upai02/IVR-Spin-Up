#include "main.h"
#include "auton.h"
#include <array>
#include <cmath>
#include <map>
#include <vector>
#include "controls.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "misc/PositionTracker.h"
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

	std::vector<std::vector<double>> skillsPathSeg1 = {{1.45, 3.45}, {1.8, 3.45}}; // reversed, facing 270
	// starting x is with front of robot on opponent low goal plane, y is against wall.

	// initTracker(skillsPathSeg1[0][0], skillsPathSeg1[0][1]);
	initTracker();
	pros::Task position_updater(update_position);
	pros::delay(5000);
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
	left_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	std::vector<std::vector<double>> skillsPathSeg1 = {{1.45, 3.45}, {1.8, 3.45}}; // reversed, facing 270
	// starting x is with front of robot on opponent low goal plane, y is against wall.

	// initTracker(skillsPathSeg1[0][0], skillsPathSeg1[0][1]);
	// imu.set_heading(270);

	// initTracker(0.0, 0.0);
	// pros::delay(1000);
		
	// SmartStop();

	// Figure 8
	// x = goes to the right (relative to starting facing forward), y = goes forward
	// clockwise increases angle value 
	std::vector<std::vector<double>> appPath = {
		{0, 0}, {-1, 0.75}, {0, 1.5}, {1, 2.25}, {0, 3}, {-1, 2.25}, {0, 1.5}, {1, 0.75}, {0, 0}
	};
	// followPath(appPath, 270, false);

	std::vector<std::vector<double>> reverseTestPath {
		{0, 0}, {1, 0}, {2, 1}, {2, 0}, {1, 0}, {0, 0}
	};
	// followPath(reverseTestPath, 89.0, true);

	std::vector<std::vector<double>> straight_path = {{0, 0}, {0, 0.75}, {0, 1.5}, {0, 2.25}};
	followPath(straight_path, 0, false);


	std::vector<std::vector<double>> skillsPathSeg2 = {{skillsPathSeg1.back()}, {1.6, 3.2}}; // forward, end facing 270. Turn to 0 after shooting.
	std::vector<std::vector<double>> skillsPathSeg3 = {{skillsPathSeg2.back()}, {1.55, 2.42}, {1.55, 2.17}}; // reversed, end facing 315
	std::vector<std::vector<double>> skillsPathSeg4 = {{skillsPathSeg3.back()}, {1.22, 1.83}, {0.7, 1.3}, {0.45, 2.11}}; // end facing goal (spin on spot)
	std::vector<std::vector<double>> skillsPathSeg5 = {{skillsPathSeg4.back()}, {1.46, 2.26}}; // pick up 3 and drive back to center shooting spot again
	// std::vector<std::vector<double>> skillsPathSeg6 = {{skillsPathSeg5.back()}, {1.9, 2.7}}; // go get first set of 3
	std::vector<std::vector<double>> skillsPathSeg6 = {{skillsPathSeg5.back()}, {2.1, 2.7}}; // go get first set of 3	
	// std::vector<std::vector<double>> skillsPathSeg7 = {{skillsPathSeg6.back()}, {1.83, 3.05}}; // go closer and shoot first set of 3
	std::vector<std::vector<double>> skillsPathSeg8 = {{skillsPathSeg6.back()}, {2.54, 2.7}}; // go get 2nd set of 3
	std::vector<std::vector<double>> skillsPathSeg9 = {{skillsPathSeg8.back()}, {1.9, 2.9}}; // shoot 2nd set of 3
	std::vector<std::vector<double>> skillsPathSeg10 = {{skillsPathSeg9.back()}, {2.1, 2.1}}; // get first solo on diagonal
	std::vector<std::vector<double>> skillsPathSeg11 = {{skillsPathSeg10.back()}, {2.7, 2.7}, {1.8, 3.4}, {1.5, 3.356}}; // get 2nd on solo diagonal + back to drop spot

	// intake on
	followPath(skillsPathSeg1, 270, true);
	followPath(skillsPathSeg2, calcGoalAngle(skillsPathSeg2.back()), false);
	// shoot
	turnToAngle(0.0, 2.0);
	followPath(skillsPathSeg3, calcGoalAngle(skillsPathSeg3.back()), true);
	// shoot
	followPath(skillsPathSeg4, calcGoalAngle(skillsPathSeg4.back()), true);
	// shoot
	turnToAngle(270.0, 2.0);
	followPath(skillsPathSeg5, calcGoalAngle(skillsPathSeg5.back()), true);
	// shoot
	followPath(skillsPathSeg6, calcGoalAngle(skillsPathSeg6.back()), true);
	// shoot
	// followPath(skillsPathSeg7, calcGoalAngle(skillsPathSeg7.back()), false);
	followPath(skillsPathSeg8, 270.0, true, true, 0.5, 45.0);
	followPath(skillsPathSeg9, calcGoalAngle(skillsPathSeg9.back()), false);
	// shoot
	followPath(skillsPathSeg10, 320, true, true, 0.5, 45.0);
	followPath(skillsPathSeg11, calcGoalAngle(skillsPathSeg11.back()), true); // 0.2 off on Y with these
	// shoot
	// SmartStop();
	stopMotors();
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
	autonomous();

	// imu.reset();
	// pros::delay(5000);

	// controls();
}
