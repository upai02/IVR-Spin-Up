#include "main.h"
#include "auton.h"
#include <array>
#include <cmath>
#include <map>
#include <vector>
#include "controls.h"
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
	// drivePID(12);

	imu.reset();
	pros::delay(5000);

	initTracker();

	/*
	std::vector<std::vector<double>> appPath = {
		{0 , 0}, {0, 2}, {1, 3}, {2, 2}, {1, 1}, {0, 2}
	};
	*/

	// std::vector<std::vector<double>> appPath = {
	// 	{0, 0}, {0, 1}, {1, 1}, {1, 0.5}, {-1, 0.5}
	// };

	// figure 8
	std::vector<std::vector<double>> appPath = {
		{0, 0}, {-1, 1}, {0, 2}, {1, 3}, {0, 4}, {-1, 3}, {0, 2}, {1, 1}, {0, 0}
	};

	// std::vector<std::vector<double>> appPath = {
	// 	{0, 0.3}, {0, 0.5}, {0, 1.5}, {0, 2.0}
	// };

	// 	followPath(std::vector<std::vector<double>> &path, double lookForwardRadius, double translationalRPM, double maxRPM)
	followPath(appPath, 0.5, 150.0, 200, 270);

	/*
	std::map<double, std::array<double, 2>> xyAutoCoords;
	// x = goes to the right (relative to starting facing forward), y = goes forward
	xyAutoCoords[0.25] = {0, 0};
	xyAutoCoords[10] = {2.5, 1};
	xyAutoCoords[20] = {-1, 1};
	xyAutoCoords[25] = {-1, -1};
	xyAutoCoords[30] = {0, 0};

	FollowXYPath xyPath = FollowXYPath(xyAutoCoords, (2.75 * 0.0254 / 2));
	xyPath.initPath();

	followXYPath(xyPath);
	*/
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
