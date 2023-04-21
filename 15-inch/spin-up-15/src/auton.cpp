#include "auton.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "shooter.h"
#include "misc/PositionTracker.h"
#include "intake.h"
#include "roller.h"
#include "endgame.h"
#include <vector>

char auton_sel = 'E';

template <typename T>
int sgn (T num) {
    if (num >= 0) {
        return 1;
    } 
    else {
        return -1;
    }
}

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
    
    double starting_speed = 200;
    double accel = 1;

    while (std::abs(left_error) > 25 || std::abs(right_error) > 25) {
        starting_speed += accel;

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

        if (fabs(left_speed) > starting_speed) {
            left_speed = starting_speed * sgn(left_speed);
        }
        if (fabs(right_speed) > starting_speed) {
            right_speed = starting_speed * sgn(right_speed);
        }

        left_side.move_velocity(left_speed);
        right_side.move_velocity(right_speed);
        pros::delay(20);
    }
    left_side.brake();
    right_side.brake();
}

void assign_min_speed (double& speed, double min_speed) {
    if (speed < 0) {
        if (fabs(speed) < min_speed) {
            speed = -min_speed;
        }
    }
    if (speed > 0) {
        if (fabs(speed) < min_speed) {
            speed = min_speed;
        }
    }
}

// could change currentHeading with imu.get_heading()
void turnPID(double deg, double kp, double ki, double kd, double max_speed, double min_speed) {
    // imu.reset();
    // std::cout << "turn PID current Heading: " << currentHeading << std::endl;
    double target = deg;
    // double error = target - currentHeading;
    double error = getAngleError(target, imu.get_heading());
    double derivative = 0;
    double integral = 0;
    double prev_error = 0;
    double speed = 0;

    int stop_counter = 0;

    while (stop_counter < 15) {
        // error = target - currentHeading;
        error = getAngleError(target, imu.get_heading());
        
        // pros::lcd::print(6, "heading: %f", imu.get_heading());
        pros::lcd::print(7, "turn PID error: %f", error);

        std::cout << "turn PID error: " << error << std::endl;
        derivative = error - prev_error;
        integral += error;
        speed = error * kp + integral * ki + derivative * kd;
        prev_error = error;

        if (speed < 0) {
            speed -= min_speed;
        }
        else {
            speed += min_speed;
        }

        speed = std::abs(speed) > max_speed ? max_speed * (speed / std::abs(speed)) : speed;

        // assign_min_speed(speed, min_speed);

        left_side.move(speed);
        right_side.move(-speed);

        if (std::abs(error) < 7) {
            stop_counter++;
        } else {
            stop_counter = 0;
        }

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
                // rai_mtr.move_voltage(0);
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
        pros::delay(20);
    }
}

pros::Task auton_task(auton_thread);

void test_auton() {
    std::vector<std::vector<double>> path = {{0, 0}, {-2, 0}, {-2, -2}};
    followPath(path, 0, true, false, false, 0.5, 10.0, 150.0, 300.0, false);
    stopMotors();
}

void auton() {

    discs_in_mag = 2;

    set_flywheel_rpm(515);
    flywheel_task.resume();
    
    auton_sel = 'I'; // intake();
    auton_task.resume();
    
	// pros::delay(500);

    drivePID(30);
    
    // intake_mtr.move_voltage(0);
    // rai_mtr.move_voltage(0);

    // turnPID(121);
    turnPID(117.5, 1, 0, 0, 40, 15);

    drivePID(6);

    pros::delay(4500);
    auton_sel = 'M';
    // auton_sel = 'R';
    release_sequence();
    pros::delay(300);

    set_flywheel_rpm(510);
    // more pathing
    auton_sel = 'I';
    // intake();

    // turnPID(135);
    turnPID(135, 1, 0, 0, 30, 15);
    drivePID(6.5);

    drivePID(-11.5);
    // turnPID(45);
    turnPID(45, 1, 0, 0, 30, 15);
    drivePID(12);
    drivePID(12);
    // pros::delay(1000);

    // turnPID(130);
    turnPID(90, 1, 0, 0, 30, 15);
    turnPID(130, 1, 0, 0, 30, 15);

    // drivePID(10.6);
    pros::delay(2000);

    auton_sel = 'M';
    // intake_mtr.move_voltage(0);
    // rai_mtr.move_voltage(0);

    // auton_sel = 'R';
    release_sequence();
    pros::delay(500);


    flywheel_task.suspend();
    auton_task.suspend();
    auton_sel = 'E';
    intake_mtr.move_voltage(0);
    rai_mtr.move_voltage(0);
    flywheel_mtr.move_voltage(0);
}

void skill_auton() {

    // turnPID(180, 1, 0, 0, 30, 15);
    drivePID(40);
    turnPID(125, 1, 0, 0, 30, 15);
    pros::delay(20*1000);
    activate_endgame();

}

void rollerAutoPATH() {
    // robot center: 35in x, 16in y

    // pros::Task position_updater(updatePosition);

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
    // const double ROLLER_START_POSITION = roller.get_position();

    // while (std::abs(SPIN_TICKS_FIRST) > std::abs(roller.get_position() - ROLLER_START_POSITION)) {
    //     roller.move_velocity(ROLLER_MOVE_VEL);
    //     pros::delay(50);
    // }
    moveMotors(0, 0);
    // roller.move_velocity(0);

    std::vector<std::vector<double>> path_to_other_roller = {{starting_position}, {1.2, 0.6}, {2.4, 0.25}, {3.0, 0.6}, {3.2, 1.2}, {3.0, 1.6}, {3.1, 2.67}};
    
    followPath(path_to_other_roller, 270, false, true, false, 0.5, 3.0); //200, 275
    moveMotors(-30, -30);
    pros::delay(WALL_WAIT_MILLISECONDS);

    // double roller_second_start_pos = roller.get_position();
    // while (std::abs(SPIN_TICKS_SECOND) > std::abs(roller.get_position() - roller_second_start_pos)) {
    //     roller.move_velocity(ROLLER_MOVE_VEL);
    //     pros::delay(50);
    // }
    moveMotors(30, 30);
    // roller.move_velocity(0);
    pros::delay(2000);
    stopMotors();

    // position_updater.suspend();
    // done!
    // pros::lcd::set_text(3, "Done!");
}

void compAutonLeftRobot() {
    // Robo dims: 15.5 (0.4) length by 12.5 (0.31) inches (width)
    // Starting pos: (1.02, 0.4)

    std::vector<double> starting_pos = {1.02, 0.4};
    std::vector<std::vector<double>> get_midline_three = {starting_pos, {1.02, 0.75}};
    followPath(get_midline_three, 0, false, false, true);
    // shoot
    pros::delay(2000);
    moveMotors(-50, -50);
    pros::delay(2000);
    std::vector<std::vector<double>> get_central_three = {{1.15, 0.5}, {1.5, 0.9}};
    followPath(get_central_three, 45, false, false, true);
    // shoot
    pros::delay(2000);
    std::vector<std::vector<double>> get_side_three = {get_central_three.back(), {2.4, 0.0}, {2.3, 1.2}};
    followPath(get_side_three, 0, false, true, true);
    // shoot
}


void compAutonRightRobot() {
    // Robo dims: 15.5 length by 12.5 inches (width)
    // Starting pos: (3.25, 2.15) facing 315 deg

    std::vector<double> starting_pos = {3.25, 2.15};
    // intake on 
    std::vector<std::vector<double>> to_mid_line_three = {starting_pos, {2.8, 2.6}};
    followPath(to_mid_line_three, 315.0, false);
    pros::delay(200);
    pros::delay(2000);
    std::vector<std::vector<double>> to_shoot_pos_one = {to_mid_line_three.back(), {3.0, 2.4}};
    followPath(to_shoot_pos_one, 315.0, true, false, true);
    pros::delay(2000);
    // shoot
    turnToAngle(225.0, 7.0);

    std::vector<std::vector<double>> to_shoot_pos_two = {to_shoot_pos_one.back(), {2.1, 1.5}};
    followPath(to_shoot_pos_two, 225, false, false, true);
    pros::delay(2000);
    // shoot
    turnToPoint(3.6, 0.6);
    std::vector<std::vector<double>> get_goal_bar_three = {to_shoot_pos_two.back(), {2.9, 1.0}, {3.3, 1.4}, {3.1, 2.7}};
    followPath(get_goal_bar_three, 280, false, false, true);
    pros::delay(2000);
    // shoot
    turnToAngle(270, 2.0);

    moveMotors(-60, -60);
    pros::delay(1000);
    // roller
    moveMotors(60, 60);
    pros::delay(500);
    stopMotors();
}