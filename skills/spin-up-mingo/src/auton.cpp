#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pros/rtos.hpp"

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

        left_front_mtr.move_velocity(powers[0]);
        right_front_mtr.move_velocity(powers[1]);
        left_back_top_mtr.move_velocity(powers[0]);
        right_back_top_mtr.move_velocity(powers[1]);
        left_back_bottom_mtr.move_velocity(powers[0]);
        right_back_bottom_mtr.move_velocity(powers[1]);

        pros::delay(20);
    }
}