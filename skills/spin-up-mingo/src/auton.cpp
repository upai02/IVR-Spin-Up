#include "auton.h"
#include "controls.h"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "robot.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include "movement.h"
#include "misc/PositionTracker.h"

void drivePID(double inches) {
    left_side.tare_position();
    right_side.tare_position();
    double target = inches * 360 / 2.25;
    double left_error = target - ((left_side.get_positions()[0] + left_side.get_positions()[1]) / 2);
    double right_error = target - ((right_side.get_positions()[0] + right_side.get_positions()[1]) / 2);
    double left_derivative = 0;
    double right_derivative = 0;
    double left_prev_error = 0;
    double right_prev_error = 0;
    double left_speed = 0;
    double right_speed = 0;
    double left_kp = 1;
    double right_kp = 1;
    double left_kd = 0;
    double right_kd = 0;
    while (std::abs(left_error) > 10 || std::abs(right_error) > 10) {
        left_error = target - ((left_side.get_positions()[0] + left_side.get_positions()[1]) / 2);
        right_error = target - ((right_side.get_positions()[0] + right_side.get_positions()[1]) / 2);
        left_derivative = left_error - left_prev_error;
        right_derivative = right_error - right_prev_error;
        left_speed = left_error * left_kp + left_derivative * left_kd;
        right_speed = right_error * right_kp + right_derivative * right_kd;
        left_prev_error = left_error;
        right_prev_error = right_error;
        left_side.move_voltage(left_speed);
        right_side.move_voltage(right_speed);
        pros::delay(20);
    }
}

void shootPID(double angle) {
    double target = angle;
    double error = target - catpot.get_angle();
    double derivative = 0;
    double prev_error = 0;
    double speed = 0;
    double integral = 0;
    double kp = 0.0564;
    double ki = 0;
    double kd = 0;
    while (std::abs(error) > 10) {
        error = target - catpot.get_angle();
        integral = integral + error;
        derivative = error - prev_error;
        speed = error * kp + integral * ki + derivative * kd;
        prev_error = error;
        catapult.move(speed);
        pros::delay(20);
    }
}

void followXYPath(FollowXYPath& xyPath) {
    while (true) {
        std::array<double, 2> powers = xyPath.executePathLoop();

        left_front_top_mtr.move_velocity(powers[0]);
        right_front_top_mtr.move_velocity(powers[1]);
        left_front_bottom_mtr.move_velocity(powers[0]);
        right_front_bottom_mtr.move_velocity(powers[1]);
        left_back_mtr.move_velocity(powers[0]);
        right_back_mtr.move_velocity(powers[1]);

        pros::delay(20);
    }
}

void rollerAuto() {
    // robot center: 35in x, 16in y

    pros::Task position_updater(update_position);

    const double SPIN_TICKS_FIRST = -800;
    const double SPIN_TICKS_SECOND = -800;
    const double ROLLER_MOVE_VEL = -30;
    const double WALL_WAIT_MILLISECONDS = 4000;
    // -2900 per spin in correct direction
    // drive up to roller:
    // catapult.brake();
    moveMotors(-60, -60);
    pros::delay(WALL_WAIT_MILLISECONDS / 2);

    std::vector<double> starting_position = {0.9, 0.21}; // 7 inches (0.18 meters) off wall
    // - back of robot touching vertical plane created by furthest edge of the 2nd foam tile into the field
    const double ROLLER_START_POSITION = roller.get_position();

    while (std::abs(SPIN_TICKS_FIRST) > std::abs(roller.get_position() - ROLLER_START_POSITION)) {
        roller.move_velocity(ROLLER_MOVE_VEL);
        pros::delay(50);
    }
    moveMotors(0, 0);
    roller.move_velocity(0);

    std::vector<std::vector<double>> path_to_other_roller = {{starting_position}, {1.2, 0.6}, {2.4, 0.25}, {3.0, 0.6}, {3.2, 1.2}, {3.0, 1.6}, {3.1, 2.7}};
    
    followPath(path_to_other_roller, 270, false, true, false, 0.5, 3.0); //200, 275
    moveMotors(-30, -30);
    pros::delay(WALL_WAIT_MILLISECONDS);

    double roller_second_start_pos = roller.get_position();
    while (std::abs(SPIN_TICKS_SECOND) > std::abs(roller.get_position() - roller_second_start_pos)) {
        roller.move_velocity(ROLLER_MOVE_VEL);
        pros::delay(50);
    }
    moveMotors(30, 30);
    roller.move_velocity(0);
    pros::delay(2000);
    stopMotors();

    position_updater.suspend();
    // done!
    pros::lcd::set_text(3, "Done!");
}

void getOneFromStack() {
    // assumes first disk has been gotten
    intake.move_voltage(0);
    moveMotors(100, 100);
    pros::delay(800);
    intake.move_voltage(12000);
    moveMotors(-100, -100);
    pros::delay(1000);
    stopMotors();
    pros::delay(200);
}

void boltSkillsAuto() {
    // left_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // left_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// right_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // right_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    imu.set_heading(270);

    pros::delay(100);

    pros::Task position_updater(update_position);
    pros::Task shooter_reseter(ResetShooterLoop);

	std::vector<std::vector<double>> skillsPathSeg1 = {{1.42, 3.4}, {1.8, 3.4}}; // reversed, facing 270
	// starting x is with front of robot on opponent low goal plane, y is against wall.
	// initTracker(0.0, 0.0);
	// pros::delay(1000);	
	// SmartStop();

	// Figure 8
	// x = goes to the right (relative to starting facing forward), y = goes forward
	// clockwise increases angle value 
	std::vector<std::vector<double>> appPath = {
		{0, 0}, {-1, 1}, {0, 2}, {1, 3}, {0, 4}, {-1, 3}, {0, 2}, {1, 1}, {0, 0}
	};
	// followPath(appPath, 0.5, 150.0, 200, 270, false, true);
	std::vector<std::vector<double>> app_path_condensed = {
		{0, 0}, {-1, 0.75}, {0, 1.5}, {1, 2.25}, {0, 3}, {-1, 2.25}, {0, 1.5}, {1, 0.75}, {0, 0}
	};
	// followPath(app_path_condensed, 270, false);

	std::vector<std::vector<double>> reverseTestPath {
		{0, 0}, {1, 0}, {2, 1}, {2, 0}, {1, 0}, {0, 0}
	};
	// followPath(reverseTestPath, 89.0, true);

	std::vector<std::vector<double>> skillsPathSeg2 = {{skillsPathSeg1.back()}, {1.45, 3.25}}; // forward, end facing 270. Turn to 0 after shooting.
	std::vector<std::vector<double>> skillsPathSeg3 = {{skillsPathSeg2.back()}, {1.4, 2.4}}; // reversed, end facing 315
	std::vector<std::vector<double>> skillsPathSeg4 = {{skillsPathSeg3.back()}, {1.2, 1.7}, {0.6, 1.2}, {0.35, 2.1}}; // end facing goal (spin on spot)
	std::vector<std::vector<double>> skillsPathSeg5 = {{skillsPathSeg4.back()}, {1.1, 2.35}, {1.46, 2.35}}; // pick up 3 and drive back to center shooting spot again
	// std::vector<std::vector<double>> skillsPathSeg6 = {{skillsPathSeg5.back()}, {1.9, 2.7}}; // go get first set of 3
	std::vector<std::vector<double>> skillsPathSeg6 = {{skillsPathSeg5.back()}, {2.1, 2.85}}; // go get first set of 3
    std::vector<std::vector<double>> skillsPathSeg7 = {{skillsPathSeg6.back()}, {1.45, 3.15}}; // go to shoot first set of 3
	// std::vector<std::vector<double>> skillsPathSeg7 = {{skillsPathSeg6.back()}, {1.83, 3.05}}; // go closer and shoot first set of 3
	std::vector<std::vector<double>> skillsPathSeg8 = {{skillsPathSeg7.back()}, {2.54, 2.7}}; // go get 2nd set of 3
	std::vector<std::vector<double>> skillsPathSeg9 = {{skillsPathSeg8.back()}, {1.9, 2.9}}; // shoot 2nd set of 3
	std::vector<std::vector<double>> skillsPathSeg10 = {{skillsPathSeg9.back()}, {2.1, 2.1}}; // get first solo on diagonal
	std::vector<std::vector<double>> skillsPathSeg11 = {{skillsPathSeg10.back()}, {2.7, 2.7}, {1.8, 3.4}, {1.5, 3.356}}; // get 2nd on solo diagonal + back to drop spot

    while (cata_limit.get_value() == 0) {
        catapult.move_velocity(65);
        pros::delay(20);
    }
    catapult.brake();

	// intake on
	intake.move_voltage(12000);
	followPath(skillsPathSeg1, 270, true);
	followPath(skillsPathSeg2, calcGoalAngle(skillsPathSeg2.back()), false, false, true);
    // intake.move_velocity(12000);
	// shoot
	shootAndWait();
	turnToAngle(350.0, 2.0);
	followPath(skillsPathSeg3, calcGoalAngle(skillsPathSeg3.back()), true, false, true, 0.5, 3.0, 100, 200);
    // shoot
    shootAndWait();
    // intake.move_voltage(12000);
	followPath(skillsPathSeg4, calcGoalAngle(skillsPathSeg4.back()), true, false, true, 0.3, 3.0, 150, 200);
	// shoot
    shootAndWait();
    // intake.move_voltage(12000);
	turnToAngle(265.0, 2.0);
	followPath(skillsPathSeg5, calcGoalAngle(skillsPathSeg5.back()), true, false, true, 0.3, 3.0, 150, 200);
	// shoot    
    shootAndWait();
	followPath(skillsPathSeg6, 230, true, false, false, 0.3, 20);

    getOneFromStack();

	followPath(skillsPathSeg7, calcGoalAngle(skillsPathSeg7.back()), false, false, true);
    shootAndWait();
    // shoot

    /*
	followPath(skillsPathSeg8, 270.0, true, true, 0.5, 45.0);
	followPath(skillsPathSeg9, calcGoalAngle(skillsPathSeg9.back()), false);
	// shoot
	followPath(skillsPathSeg10, 320, true, true, 0.5, 45.0);
	followPath(skillsPathSeg11, calcGoalAngle(skillsPathSeg11.back()), true); // 0.2 off on Y with these
    // shoot
	*/
	// SmartStop();

    std::vector<std::vector<double>> drive_to_endgame = {skillsPathSeg7.back(), {3.0, 3.2}};
    followPath(drive_to_endgame, 225, true, true, false, 0.5, 5.0);
    intake.move_voltage(0);
    release_endgame_spools();

	intake.move_voltage(0);
	stopMotors();

    position_updater.suspend();
    shooter_reseter.suspend();
}

void boltEndgameAuto() {
    std::vector<double> start_position = {0.89, 0.4}; // 7 inches (0.18 meters) off wall
    // - back of robot touching vertical plane created by furthest edge of the 2nd foam tile into the field

    std::vector<double> post_roller_pos = {0.9, 0.21}; // where the robot will be after doing the roller
    std::vector<double> endingPoint = {0.6, 0.6};

    std::vector<std::vector<double>> line_seg_one = {post_roller_pos, {0.9, 0.6}};
    std::vector<std::vector<double>> line_seg_two = {line_seg_one.back(), {endingPoint}};

    pros::Task position_updater(update_position);

    const double SPIN_TICKS_FIRST = -1300;
    const double SPIN_TICKS_SECOND = -800;
    const double ROLLER_MOVE_VEL = -30;
    const double WALL_WAIT_MILLISECONDS = 4000;
    // -2900 per spin in correct direction
    // drive up to roller:
    moveMotors(-60, -60);
    pros::delay(WALL_WAIT_MILLISECONDS / 2);

    const double ROLLER_START_POSITION = roller.get_position();

    while (std::abs(SPIN_TICKS_FIRST) > std::abs(roller.get_position() - ROLLER_START_POSITION)) {
        roller.move_velocity(ROLLER_MOVE_VEL);
        pros::delay(50);
    }
    moveMotors(0, 0);
    roller.move_velocity(0);

    followPath(line_seg_one, 0, false);
    turnToAngle(90, 5);
    followPath(line_seg_two, 45, true, true);

    pros::delay(40000);
    release_endgame_spools();

    position_updater.suspend();
}

void StraightPathTest() {
    left_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    pros::Task position_updater(update_position);
	std::vector<std::vector<double>> straight_path = {{0, 0}, {0, 0.1}, {0, 0.735}, {0, 0.79}, {0, 0.85}, {0, 2.25}};
    std::vector<std::vector<double>> diag_path = {{0, 0}, {0.5, 0.5}, {0.5, 1}, {0, 1.5}, {0, 2}};
	// followPath(diag_path, 0, false, false, 0.2, 3.0, 37.5, 50);
    followPath(diag_path, 0, false, false, false, 0.5, 3.0, 350, 450);
    position_updater.suspend();
    SmartStop();
}