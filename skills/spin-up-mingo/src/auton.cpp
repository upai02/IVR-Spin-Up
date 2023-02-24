#include "auton.h"
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

    const double SPIN_TICKS_FIRST = -600;
    const double SPIN_TICKS_SECOND = -700;
    const double ROLLER_MOVE_VEL = -25;
    const double WALL_WAIT_MILLISECONDS = 2500;
    // -2900 per spin in correct direction
    // drive up to roller:
    moveMotors(-60, -60);
    pros::delay(WALL_WAIT_MILLISECONDS);
    moveMotors(0, 0);

    std::vector<double> starting_position = {0.9, 0.21}; // 7 inches (0.18 meters) off wall
    // - back of robot touching vertical plane created by furthest edge of the 2nd foam tile into the field
    const double ROLLER_START_POSITION = roller.get_position();

    while (std::abs(SPIN_TICKS_FIRST) > std::abs(roller.get_position() - ROLLER_START_POSITION)) {
        roller.move_velocity(ROLLER_MOVE_VEL);
        pros::delay(50);
    }
    roller.move_velocity(0);

    std::vector<std::vector<double>> path_to_other_roller = {{starting_position}, {1.2, 0.5}, {2.4, 0.25}, {3.4, 0.3}, {3.2, 2.7}};
    
    followPath(path_to_other_roller, 270, false, true);
    moveMotors(-60, -60);
    pros::delay(WALL_WAIT_MILLISECONDS);
    stopMotors();

    double roller_second_start_pos = roller.get_position();
    while (std::abs(SPIN_TICKS_SECOND) > std::abs(roller.get_position() - roller_second_start_pos)) {
        roller.move_velocity(ROLLER_MOVE_VEL);
        pros::delay(50);
    }
    roller.move_velocity(0);
    // done!
    pros::lcd::set_text(3, "Done!");
}