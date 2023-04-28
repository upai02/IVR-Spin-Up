#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "shooter.h"
#include "main.h"
#include "misc/PositionTracker.h"
#include "intake.h"
#include "roller.h"
#include "endgame.h"
#include <vector>

char auton_sel = 'E'; // initialize auton_sel to E (do nothing)
const double BASE_FLYWHEEL_RPM = 300.0; // distance for 2 meters (need to update). going to assume rpm scales linearly with distance and hope
const double BASE_RPM_DIST = 2.0;

template <typename T>
int sgn (T num) {
    if (num >= 0) {
        return 1;
    } 
    else {
        return -1;
    }
}

// pid to drive straight, just uses motor encoders
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

// constrains speed to be above the min_speed
void assign_min_speed(double& speed, double min_speed) {
    if (fabs(speed) < min_speed) {
        speed = min_speed * sgn(speed);
    }
}
// constrains speed to be below the max_speed
void assign_max_speed(double& speed, double max_speed) {
    if (fabs(speed) > max_speed) {
        speed = max_speed * sgn(speed);
    }
}

// simpler turn control loop using imu
void turnP(double deg, double kp, double min_speed, double max_speed) {
    double target = deg;
    double error = getAngleError(target, imu.get_heading());
    double speed = 0;

    int stop_counter = 0;
    while (stop_counter < 15) {
        error = getAngleError(target, imu.get_heading());
        speed = error * kp;
        assign_min_speed(speed, min_speed);
        assign_max_speed(speed, max_speed);

        left_side.move(speed);
        right_side.move(-speed);

        if (fabs(error) < 3) {
            stop_counter++;
        } else {
            stop_counter = 0;
        }

        pros::delay(20);
    }
    left_side.brake();
    right_side.brake();
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

// control loop to keep flywheel at a certain rpm
void shootPF(double rpm) {
    const double kF = 26;
    const double kP = 0.3;
    double error = rpm - get_flywheel_rpm();
    double power = kF * rpm + kP * error;
    while (std::abs(error) > 10) {
        error = rpm - get_flywheel_rpm();
        power = kF * rpm + kP * error;
        flywheel.move_voltage(power);
        pros::delay(20);
    }
}

// helpful functions for auton
void auton_thread() {
    while(1) {
        switch (auton_sel) {
            case 'I':
                intake_auton();
                break;
            case 'S':
                spin_roller();
                break;
            case 'M':
                intake_mtr.move_voltage(0);
                // rai_mtr.move_voltage(0);
                break;
            case 'R':
                release_sequence();
                break;
            case 'r':
                release_discs();
                break;
            case 'o':
                release_discs();
                outtake();
                break;
            case 'O':
                outtake();
                break;
            default:
                intake_mtr.move_voltage(0);
                rai_mtr.move_voltage(0);
                break;
        }
        pros::delay(20);
    }
}


void test_auton() {
    std::vector<std::vector<double>> path = {{0, 0}, {-2, 0}, {-2, -2}};
    followPath(path, 0, true, false, false, 0.5, 10.0, 150.0, 300.0, false);
    stopMotors();
}

void auton() {

    pros::Task auton_task(auton_thread);
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
    flywheel.move_voltage(0);
}

void skill_auton() {

    // turnPID(180, 1, 0, 0, 30, 15);
    drivePID(40);
    turnPID(125, 1, 0, 0, 30, 15);
    pros::delay(20*1000);
    // activate_endgame();

}

// Robot needs to be close to and facing roller before calling this.
void spin_roller_auton() {
    auton_sel = 'E';
    moveMotors(-75, -75);
    pros::delay(1000);
    moveMotors(-50, -50);
    auton_sel = 'S';
    pros::delay(250);
    auton_sel = 'E';
    moveMotors(75, 75);
    pros::delay(750);
    stopMotors();
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
    // Starting pos: (0.93, 0.46)'

    pros::Task auton_task(auton_thread);
    discs_in_mag = 0;
    toggle_angle_changer(); // first call, put intake out
    // should update to use shootPF function and thread
    set_flywheel_rpm(BASE_FLYWHEEL_RPM);
    flywheel_task.resume();

    auton_sel = 'I';
    std::vector<double> starting_pos = {0.93, 0.46};
    std::vector<std::vector<double>> get_midline_three = {starting_pos, {starting_pos[0], 0.75}};
    followPath(get_midline_three, 0, false, false, false, 0.5, 3.0, 200.0, 450.0, 25.0);
    toggle_angle_changer(); // second call, back to good position

    // shoot, 2.85 meters
    // pros::delay(2000);
    set_flywheel_rpm(BASE_FLYWHEEL_RPM * static_cast<int>(2.85/BASE_RPM_DIST));

    moveMotors(-100, -100);
    pros::delay(1200);
    auton_sel = 'O';
    turnToPoint();
    auton_sel = 'o';
    pros::delay(1000);
    auton_sel = 'I';
    set_flywheel_rpm(BASE_FLYWHEEL_RPM * static_cast<int>(2.55/BASE_RPM_DIST));
    std::vector<std::vector<double>> get_central_three = {{starting_pos[0], 0.5}, {1.55, 0.95}};
    followPath(get_central_three, 45, false, false, true);
    // shoot, 2.55 meters
    auton_sel = 'o';
    pros::delay(1000);
    // pros::delay(2000);
    auton_sel = 'E';
    turnToAngle(135, 10.0);
    auton_sel = 'I';
    set_flywheel_rpm(BASE_FLYWHEEL_RPM * static_cast<int>(2.75/BASE_RPM_DIST));
    std::vector<std::vector<double>> get_side_three = {get_central_three.back(), {2.05, 0.1}, {2.05, 0.6}, {2.27, 1.3}};
    followPath(get_side_three, 0, false, false, false, 0.5, 3.0, 125.0);
    auton_sel = 'O';
    turnToPoint();
    // shoot, 2.75 meters
    auton_sel = 'o';
    pros::delay(1000);
    // go to default
    auton_sel = 'O';
    // do roller
    std::vector<std::vector<double>> to_roller = {get_side_three.back(), {1.5, 0.8}, {1.0, 0.6}};
    followPath(to_roller, 235, false, false, false, 0.5, 20.0);
    turnToAngle(0, 4.0);
    // roller
    spin_roller_auton();
    // if shot here it would be ~2.85 meters
    auton_sel = 'E';
    flywheel_task.suspend();
    set_flywheel_rpm(0);
    rai_mtr.move_voltage(0);
    auton_task.suspend();
}


// UNTESTED
void compAutonRightRobot() {
    // Robo dims: 15.5 length by 12.5 inches (width)
    // Starting pos: (3.25, 2.15) facing 315 deg

    pros::Task auton_task(auton_thread);
    discs_in_mag = 0;
    toggle_angle_changer(); // first call, put intake out
    set_flywheel_rpm(BASE_FLYWHEEL_RPM);
    flywheel_task.resume();

    std::vector<double> starting_pos = {3.25, 2.15};
    // intake on
    auton_sel = 'I';
    std::vector<std::vector<double>> to_mid_line_three = {starting_pos, {2.83, 2.57}};
    followPath(to_mid_line_three, 315.0, false);
    toggle_angle_changer(); // second call, back to good position
    pros::delay(200);
    // no shoot call here
    set_flywheel_rpm(BASE_FLYWHEEL_RPM * static_cast<int>(2.72/BASE_RPM_DIST));
    std::vector<std::vector<double>> to_shoot_pos_one = {to_mid_line_three.back(), {3.0, 2.4}};
    followPath(to_shoot_pos_one, 315.0, true, false, true);
    // shoot, 2.72 meters
    auton_sel = 'o';
    pros::delay(1000);
    turnToAngle(225.0, 7.0);
    auton_sel = 'I';
    set_flywheel_rpm(BASE_FLYWHEEL_RPM * static_cast<int>(2.4/BASE_RPM_DIST));
    std::vector<std::vector<double>> to_shoot_pos_two = {to_shoot_pos_one.back(), {2.1, 1.65}};
    followPath(to_shoot_pos_two, 225, false, false, true);
    // shoot, 2.4 meters
    auton_sel = 'o';
    pros::delay(1000);
    turnToPoint(3.6, 1.6);
    auton_sel = 'I';
    set_flywheel_rpm(BASE_FLYWHEEL_RPM * static_cast<int>(2.74/BASE_RPM_DIST));
    std::vector<std::vector<double>> get_goal_bar_three = {to_shoot_pos_two.back(), {2.9, 1.45}, {3.1, 1.6}, {3.3, 2.78}};
    followPath(get_goal_bar_three, 280, false, true, true);
    // shoot, 2.74 meters
    auton_sel = 'o';
    pros::delay(1000);
    // turn to roller
    turnToAngle(270, 2.0);

    // roller
    spin_roller_auton();

    stopMotors();
    auton_sel = 'E';
    flywheel_task.suspend();
    set_flywheel_rpm(0);
    rai_mtr.move_voltage(0);
    auton_task.suspend();
}
