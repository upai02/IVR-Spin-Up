#include "auton.h"
#include "main.h"

void drivePID(double inches) {
    left_side.tare_position();
    right_side.tare_position();
    double target = inches * 360 / (3.25*2*M_PI);
    double left_error = target - ((left_side.get_positions()[0] + left_side.get_positions()[1] + left_side.get_positions()[2]) / 3);
    double right_error = target - ((right_side.get_positions()[0] + right_side.get_positions()[1] + left_side.get_positions()[2]) / 3);
    double left_derivative = 0;
    double right_derivative = 0;
    double left_prev_error = 0;
    double right_prev_error = 0;
    double left_integral = 0;
    double right_integral = 0;
    double left_speed = 0;
    double right_speed = 0;
    double left_kp = 0.25;
    double right_kp = 0.25;
    double left_ki = 0;
    double right_ki = 0;
    double left_kd = 0;
    double right_kd = 0;
    while (std::abs(left_error) > 10 || std::abs(right_error) > 10) {
        left_error = target - ((left_side.get_positions()[0] + left_side.get_positions()[1] + left_side.get_positions()[2]) / 3);
        right_error = target - ((right_side.get_positions()[0] + right_side.get_positions()[1] + right_side.get_positions()[2]) / 3);
        left_derivative = left_error - left_prev_error;
        right_derivative = right_error - right_prev_error;
        left_integral += left_error;
        right_integral += right_error;
        left_prev_error = left_error;
        right_prev_error = right_error;
        left_speed = left_error * left_kp + left_integral * left_ki + left_derivative * left_kd;
        right_speed = right_error * right_kp + right_integral * right_ki + right_derivative * right_kd;
        left_side.move(left_speed);
        right_side.move(right_speed);
        pros::delay(20);
    }
}

void turnPID(double degrees) {
    imu.reset();
    double target = degrees;
    double error = target - imu.get_rotation();
    double derivative = 0;
    double prev_error = 0;
    double speed = 0;
    double kp = 1;
    double kd = 0;
    while (std::abs(error) > 10) {
        error = target - imu.get_rotation();
        derivative = error - prev_error;
        speed = error * kp + derivative * kd;
        prev_error = error;
        left_side.move(-speed);
        right_side.move(speed);
        pros::delay(20);
    }
}

void shootPID(double rpm) {
    double target = rpm * 20;
    double error = target - flywheel_mtr.get_voltage();
    double derivative = 0;
    double prev_error = 0;
    double sum = 0;
    double speed = 0;
    double kp = 0.1;
    double ki = 0;
    double kd = 0;
    while (std::abs(error) > 10) {
        error = target - flywheel_mtr.get_actual_velocity();
        derivative = error - prev_error;
        prev_error = error;
        sum += error;
        speed = error * kp + sum * ki + derivative * kd;
        flywheel_mtr.move_voltage(speed);
        pros::delay(20);
    }
}

void shootPF(double rpm) {
    double kF = 12000/600.0; // 600 is the max motor rpm, 12000 is the max voltage
    double kP = 0.07;
    double error = rpm - flywheel_mtr.get_actual_velocity();
    double power = kF * rpm + kP * error;
    while (std::abs(error) > 10) {
        error = rpm - flywheel_mtr.get_actual_velocity();
        power += error * kF + error * kP;
        flywheel_mtr.move_voltage(power);
        pros::delay(20);
    }
}

void auton_thread() {
    while(1) {
        shootPF(450);
    }

}

void auton() {
    pros::Task auton_task(auton_thread);
    auton_task.suspend();
    auton_task.resume();
    drivePID(24);
    auton_task.suspend();
}