#include "controls.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "misc/PositionTracker.h"
#include "movement.h"
#include <cmath>
#include <string>

using namespace pros;

int mode = 2;
double p = 1.0;
const double ROLLER_VELOCITY = -70;
const double CATAPULT_VELOCITY = 65;
const double VOLTAGE_SCALE = 11000;
const double INPUT_SCALE_POWER = 1.5;
const double VOLTAGE_DEADZONE = 400;

double normalize_joystick(double input) {
  return input / 127.0;
}

const double sin_scale_factor = 2.9;
double sin_scale(double input) {
  return copysign(pow(sin((M_PI/2) * fabs(input)), sin_scale_factor), input);
}

double power_inputs(double input, double power) {
  return copysign(pow(std::abs(input), power), input);
}

double voltage_deadzone(double input) {
    if (std::abs(input) < 400) {
        input = 0.0;
    }
    return input;
}

// Regular tank drive with square scaling
void tank_drive() {
  double left = power_inputs(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), INPUT_SCALE_POWER);
  double right = power_inputs(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)), INPUT_SCALE_POWER);
  left_side.move_voltage(voltage_deadzone(left * VOLTAGE_SCALE));
  right_side.move_voltage(voltage_deadzone(right * VOLTAGE_SCALE));
}
// Regular arcade drive with square scaling
void arcade_drive(bool shooterForward) {
  double forward = power_inputs(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), INPUT_SCALE_POWER);
  double turn = power_inputs(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), INPUT_SCALE_POWER);
  double leftInput = (shooterForward) ? voltage_deadzone((forward + turn) * VOLTAGE_SCALE) : -voltage_deadzone((forward - turn) * VOLTAGE_SCALE);
  double rightInput = (shooterForward) ? voltage_deadzone((forward - turn) * VOLTAGE_SCALE) : -voltage_deadzone((forward + turn) * VOLTAGE_SCALE);
  left_side.move_voltage(leftInput);
  right_side.move_voltage(rightInput);
}
// Hybrid arcade drive with square scaling
void hybrid_drive() {
  double forward = power_inputs(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double turn = power_inputs(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
  left_side.move_voltage((forward + turn) * VOLTAGE_SCALE);
  right_side.move_voltage((forward - turn) * VOLTAGE_SCALE);
}
// Mecanum drive with square scaling
// void mecanum() {
//     int fwd = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
//     int strafe = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
//     int turn = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
//     left_front_mtr.move(fwd + strafe + turn);
//     right_front_mtr.move(fwd - strafe - turn);
//     left_back_mtr.move(fwd - strafe + turn);
//     right_back_mtr.move(fwd + strafe - turn);
// }

void group_stop(pros::motor_brake_mode_e b) {
    left_front_top_mtr.set_brake_mode(b);
    left_front_top_mtr.brake();
    right_front_top_mtr.set_brake_mode(b);
    right_front_top_mtr.brake();
    left_front_bottom_mtr.set_brake_mode(b);
    left_front_bottom_mtr.brake();
    right_front_bottom_mtr.set_brake_mode(b);
    right_front_bottom_mtr.brake();
    left_back_mtr.set_brake_mode(b);
    left_back_mtr.brake();
    right_back_mtr.set_brake_mode(b);
    right_back_mtr.brake();
}

void move_with_assigned_speed(double xVel, double yVel, double turnVel) {
    left_front_top_mtr.move_velocity(yVel + xVel - turnVel);
    left_front_bottom_mtr.move_velocity(yVel - xVel - turnVel);
    left_back_mtr.move_velocity(yVel - xVel - turnVel);
    right_front_top_mtr.move_velocity(yVel - xVel + turnVel);
    right_front_bottom_mtr.move_velocity(yVel + xVel + turnVel);
    right_back_mtr.move_velocity(yVel + xVel + turnVel);
}

void wannabeSwerve() {
    double analogX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    double analogY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    double joystickDistFromCenter = sqrt(pow(analogX, 2) + pow(analogY, 2));

    double robotAngle = imu.get_heading();
    double desiredAngle;

    // maintain robot angle if no controller input
    if (joystickDistFromCenter > 1) {
        desiredAngle = atan2(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X), master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
        desiredAngle *= (180/M_PI);
    } else {
        desiredAngle = robotAngle;
    }

    // switch domain to [0, 360]
    if (desiredAngle < 0) {
        desiredAngle += 360;
    }

    double angleDifference = desiredAngle - robotAngle;

    // optimize
    if (angleDifference > 180) {
        angleDifference -= 360;
    } else if (angleDifference < -180) {
        angleDifference += 360;
    }

    double power = angleDifference * p;
    double desiredTranslation = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

    if (angleDifference > 5) {
        desiredTranslation /= 2;
    }

    left_front_top_mtr.move(power + desiredTranslation);
    left_front_bottom_mtr.move(power + desiredTranslation);
    left_back_mtr.move(power + desiredTranslation);
    right_front_top_mtr.move(-power + desiredTranslation);
    right_front_bottom_mtr.move(-power + desiredTranslation);
    right_back_mtr.move(-power + desiredTranslation);
}

void shootAndWait() {
    while (cata_limit.get_value() == 1) {
        catapult.move_velocity(CATAPULT_VELOCITY);
    }
    while (cata_limit.get_value() == 0) {
        catapult.move_velocity(CATAPULT_VELOCITY);
    } 
    catapult.brake();
}

bool shooterLoop(bool shoot_active) {
    if (shoot_active) {
        if (cata_limit.get_value() != 1) {
            shoot_active = false;
        } else {
            catapult.move_velocity(CATAPULT_VELOCITY);
        }
    } else {
        if (cata_limit.get_value() == 0) {
            catapult.move_velocity(CATAPULT_VELOCITY);
        } else {
            catapult.brake();
        }
    }
    return shoot_active;
}

void ResetShooterLoop() {
    if (cata_limit.get_value() == 0) {
        catapult.move_velocity(CATAPULT_VELOCITY);
    } else {
        catapult.brake();
    }
}

void ShootDisksBlocking() {
    while (cata_limit.get_value() == 1) {
        catapult.move_velocity(CATAPULT_VELOCITY);
        pros::delay(20);
    }
}

void release_endgame_spools() {
    endgame_release.set_value(1);
    pros::delay(1000);
    endgame_release.set_value(0);
}

void controls() {
    bool shoot_active = false;

    left_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while(1) {
        // positionX_mutex.take();
        arcade_drive(false);

        pros::lcd::set_text(2, "cata_limit: " + std::to_string(cata_limit.get_value()));

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_voltage(12000);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_voltage(-12000);
        } else {
            intake.brake();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            shoot_active = true;
        }
        shoot_active = shooterLoop(shoot_active);

        /*
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            turnToPoint();
        }
        */

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            roller.move_velocity(ROLLER_VELOCITY);
        } else {
            roller.brake();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            release_endgame_spools();
        }

        // positionX_mutex.give();
        pros::delay(50);
    }
}

/*
Harith's desired controls:

 - intake in: R1
 - intake out: R2
 - shoot: L1
 - endgame (two buttons required): as is if we can get a scuff, down and left arrows if we can't get a scuff
 - roller: L2

*/