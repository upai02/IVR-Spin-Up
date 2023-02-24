#include "auton.h"
#include "main.h"
#include "shooter.h"
#include "misc/PositionTracker.h"
#include "intake.h"
#include "movement.h"

void drivePID(double inches) {
    left_side.tare_position();
    right_side.tare_position();
    double target = inches * 360 / (2.75 * M_PI);
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
    double left_ki = 0.0033;
    double right_ki = 0.0033;
    double left_kd = 0.01;
    double right_kd = 0.01;
    while (std::abs(left_error) > 25 || std::abs(right_error) > 25) {
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
        left_speed = std::abs(left_speed) > 400 ? 400 * (left_speed / std::abs(left_speed)) : left_speed;
        right_speed = std::abs(right_speed) > 400 ? 400 * (right_speed / std::abs(right_speed)) : right_speed;
        left_side.move_velocity(left_speed);
        right_side.move_velocity(right_speed);
        pros::delay(20);
    }
    left_side.brake();
    right_side.brake();
}

void turnPID(double degrees) {
    imu.reset();
    double target = degrees;
    double error = target - imu.get_rotation();
    double derivative = 0;
    double prev_error = 0;
    double speed = 0;
    double kp = 0.5;
    double kd = 0;
    while (std::abs(error) > 10) {
        error = target - imu.get_rotation();    
        derivative = error - prev_error;
        speed = error * kp + derivative * kd;
        prev_error = error;
        // speed = std::abs(speed) > 100 ? 100 * (speed / std::abs(speed)) : speed;
        left_side.move(-speed);
        right_side.move(speed);
        pros::delay(20);
    }
}

void shootPF(double rpm) {
    double kF = 20.8;
    double kP = 0.49;
    double error = rpm - flywheel_mtr.get_actual_velocity();
    double power = kF * rpm + kP * error;
    while (std::abs(error) > 10) {
        error = rpm - flywheel_mtr.get_actual_velocity();
        power = kF * rpm + kP * error;
        flywheel_mtr.move_voltage(power);
        pros::delay(20);
    }
}

void auton() {
    gps.initialize_full(0, 0, 0, 0, 0);
    pros::delay(3000);
    // gps.set_rotation(-init_heading);
    std::cout << "start turn pid" << std::endl;
    turnPID(90);
    std::cout << "end turn pid" << std::endl;

    std::vector<std::vector<double>> initialPath = {{1.8, 0.15}, {1.8, 1.22}};

    std::vector<std::vector<double>> path = {};
    for (int i = 0; i < 10; i++) {
        path.push_back({1.8, i + 0.15});
    }    

    initTracker(initialPath[0][0], initialPath[0][1]);

    pros::Task odom(updatePosition);
    std::cout << "follow Path now" << std::endl;
    followPath(initialPath, 0, false);
    
    // start auto with 2 discs
    discs_in_mag = 2;

    // set_flywheel_rpm(500);
    // flywheel_task.resume();
    // intake_mtr.move_voltage(12000);
    // rai_mtr.move_voltage(6000);
    // discs_in_mag = 2;
    // intake();
    // flywheel_task.resume();
    // intake_mtr.move_voltage(12000);
    // rai_mtr.move_voltage(6000);

    // moveMotors(30, 30);
    // pros::delay(100000);

    // followPath(initialPath, 90, false);
    turnPID(180);

    // drivePID(48);
    // pros::delay(2000);
    // intake_mtr.move_voltage(0);
    // rai_mtr.move_voltage(0);
    // pros::delay(1000);

    // toggle_mag_piston();
    // release_discs();
    // pros::delay(7000);

    // left_side.move_relative(200, 100);
    // right_side.move_relative(-200, -100);
    // pros::delay(500);
    // left_side.move_velocity(0);
    // right_side.move_velocity(0);
    // toggle_mag_piston();
    // pros::delay(3000);
    // rai_mtr.move_voltage(-9000);
    // pros::delay(6000);
    // rai_mtr.move_voltage(0);
    // pros::delay(1000);
    // rai_mtr.move_velocity(0);
    // flywheel_task.suspend();
    // flywheel_mtr.brake();
}