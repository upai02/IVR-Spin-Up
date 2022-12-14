#include "auton.h"
#include "main.h"

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
        left_side.move(left_speed);
        right_side.move(right_speed);
        pros::delay(20);
    }
}