#include "auton.h"
#include "shooter.h"
#include "main.h"
#include "misc/PositionTracker.h"
#include "intake.h"
#include "movement.h"
#include "roller.h"
#include "endgame.h"

char auton_sel = 'E'; // initialize auton_sel to E (do nothing)

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
    const double kF = 20.8;
    const double kP = 0.49;
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
