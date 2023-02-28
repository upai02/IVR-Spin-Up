#include "auton.h"
#include "main.h"
#include "shooter.h"
#include "misc/PositionTracker.h"
#include "intake.h"
#include "movement.h"
#include "roller.h"

char auton_sel = 'E';

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

// void drivePIDodom(double meters) {
//     // left_side.tare_position();
//     // right_side.tare_position();
//     double 
//     double left_error = target - ((left_side.get_positions()[0] + left_side.get_positions()[1] + left_side.get_positions()[2]) / 3);
//     double right_error = target - ((right_side.get_positions()[0] + right_side.get_positions()[1] + left_side.get_positions()[2]) / 3);
//     double left_derivative = 0;
//     double right_derivative = 0;
//     double left_prev_error = 0;
//     double right_prev_error = 0;
//     double left_integral = 0;
//     double right_integral = 0;
//     double left_speed = 0;
//     double right_speed = 0;
//     double left_kp = 0.25;
//     double right_kp = 0.25;
//     double left_ki = 0.0033;
//     double right_ki = 0.0033;
//     double left_kd = 0.01;
//     double right_kd = 0.01;
//     while (std::abs(left_error) > 25 || std::abs(right_error) > 25) {
//         left_error = target - ((left_side.get_positions()[0] + left_side.get_positions()[1] + left_side.get_positions()[2]) / 3);
//         right_error = target - ((right_side.get_positions()[0] + right_side.get_positions()[1] + right_side.get_positions()[2]) / 3);
//         left_derivative = left_error - left_prev_error;
//         right_derivative = right_error - right_prev_error;
//         left_integral += left_error;
//         right_integral += right_error;
//         left_prev_error = left_error;
//         right_prev_error = right_error;
//         left_speed = left_error * left_kp + left_integral * left_ki + left_derivative * left_kd;
//         right_speed = right_error * right_kp + right_integral * right_ki + right_derivative * right_kd;
//         left_speed = std::abs(left_speed) > 400 ? 400 * (left_speed / std::abs(left_speed)) : left_speed;
//         right_speed = std::abs(right_speed) > 400 ? 400 * (right_speed / std::abs(right_speed)) : right_speed;
//         left_side.move_velocity(left_speed);
//         right_side.move_velocity(right_speed);
//         pros::delay(20);
//     }
//     left_side.brake();
//     right_side.brake();
// }

// could change currentHeading with imu.get_heading()
void turnPID(double deg) {
    // imu.reset();
    // std::cout << "turn PID current Heading: " << currentHeading << std::endl;
    double target = deg;
    // double error = target - currentHeading;
    double error = getAngleError(target, imu.get_heading());
    double derivative = 0;
    double integral = 0;
    double prev_error = 0;
    double speed = 0;
    const double kp = 1; // 1.2, 0.7
    const double ki = 0; // 0.0069
    const double kd = 0; // 0.04
    while (std::abs(error) > 10) {
        // error = target - currentHeading;
        error = getAngleError(target, imu.get_heading());

        pros::lcd::print(6, "turn PID error: %f", error);

        std::cout << "turn PID error: " << error << std::endl;
        derivative = error - prev_error;
        integral += error;
        speed = error * kp + integral * ki + derivative * kd;
        prev_error = error;
        speed = std::abs(speed) > 400 ? 400 * (speed / std::abs(speed)) : speed;
        left_side.move(speed);
        right_side.move(-speed);
        pros::delay(20);
    }
    left_side.brake();
    right_side.brake();
}

double getAngleError(double target, double currHeading) {
    double error = target - currHeading;
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }
    return error;
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

void auton_thread() {
    while(1) {
        switch (auton_sel) {
            case 'I':
                intake();
                break;
            case 'S':
                spin_roller();
                break;
            case 'M':
                intake_mtr.move_voltage(0);
                rai_mtr.move_voltage(0);
                toggle_mag_piston();
                break;
            case 'm':
                toggle_mag_piston();
            case 'R':
                release_sequence();
                break;
            default:
                intake_mtr.move_voltage(0);
                rai_mtr.move_voltage(0);
                break;
        }
    }
}

pros::Task auton_task(auton_thread);

void auton() {

	// initTracker(0, 0);
	// pros::Task odom(updatePosition);
    // pros::delay(1000);
    // imu.reset();

    // HERERERERERERERERERERE
    // start_flywheel_task = true;

    discs_in_mag = 2;

    set_flywheel_rpm(510);
    flywheel_task.resume();
    // intake();
    auton_sel = 'I';
    auton_task.resume();
    
	pros::delay(500);

    drivePID(30);
    pros::delay(2000);
    // spin_roller();
    auton_sel = 'S';
    pros::delay(1000);
    auton_sel = 'M';
    // intake_mtr.move_voltage(0);
    // rai_mtr.move_voltage(0);
    // toggle_mag_piston();

    turnPID(121);
    pros::delay(500);

    drivePID(6);
    pros::delay(500);

    auton_sel = 'R';
    // release_sequence();
    pros::delay(800);

    set_flywheel_rpm(505);
    // more pathing
    auton_sel = 'I';
    // intake();
    // intake_mtr.move_voltage(12000);
    // rai_mtr.move_voltage(4000);
    turnPID(135);
    pros::delay(600);
    drivePID(4.9);
    pros::delay(600);

    drivePID(-15);
    pros::delay(600);
    turnPID(45);
    pros::delay(600);
    drivePID(20);
    pros::delay(600);


    turnPID(130);
    pros::delay(600);


    drivePID(10.6);
    pros::delay(1000);

    auton_sel = 'M';
    // intake_mtr.move_voltage(0);
    // rai_mtr.move_voltage(0);
    // toggle_mag_piston();
    pros::delay(800);

    auton_sel = 'R';
    // release_sequence();
    pros::delay(600);


    flywheel_task.suspend();
    auton_task.suspend();
    auton_sel = 'E';
    intake_mtr.move_voltage(0);
    rai_mtr.move_voltage(0);
    flywheel_mtr.move_voltage(0);


    // HEREREREREREREREREREREERERERERERERERERERERERERER



    // pros::delay(2000);
    // std::cout << "GOT HEREERE __________" << std::endl;
    // std::vector<std::vector<double>> initialPath = {{0.0, 0.0}, {0.0, 0.1}};
    // followPath(initialPath, 180, false);
    // pros::lcd::print(0, "DONE __________");
    // std::cout << "got hGOTE REHREHER ER  ere 2"  << std::endl;
    // pros::delay(3000);
    // left_side.move_voltage(0);
    // right_side.move_voltage(0);
    

    // gps.initialize_full(0, 0, 0, 0, 0);
    // pros::delay(3000);
    // // gps.set_rotation(-init_heading);
    // std::cout << "start turn pid" << std::endl;
    // turnPID(90);
    // std::cout << "end turn pid" << std::endl;

    // std::vector<std::vector<double>> initialPath = {{1.8, 0.15}, {1.8, 1.22}};

    // std::vector<std::vector<double>> path = {};
    // for (int i = 0; i < 10; i++) {
    //     path.push_back({1.8, i + 0.15});
    // }    

    // initTracker(initialPath[0][0], initialPath[0][1]);

    // pros::Task odom(updatePosition);
    // std::cout << "follow Path now" << std::endl;
    // followPath(initialPath, 0, false);
    
    // // start auto with 2 discs
    // discs_in_mag = 2;

    // // set_flywheel_rpm(500);
    // // flywheel_task.resume();
    // // intake_mtr.move_voltage(12000);
    // // rai_mtr.move_voltage(6000);
    // // discs_in_mag = 2;
    // // intake();
    // // flywheel_task.resume();
    // // intake_mtr.move_voltage(12000);
    // // rai_mtr.move_voltage(6000);

    // // moveMotors(30, 30);
    // // pros::delay(100000);

    // // followPath(initialPath, 90, false);
    // turnPID(180);

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