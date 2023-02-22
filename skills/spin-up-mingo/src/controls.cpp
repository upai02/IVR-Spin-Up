#include "controls.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "misc/PositionTracker.h"

using namespace pros;

int mode = 2;
double p = 1.0;

double normalize_joystick(double input) {
  return input / 127.0;
}

const double sin_scale_factor = 2.9;
double sin_scale(double input) {
  return copysign(pow(sin((M_PI/2) * fabs(input)), sin_scale_factor), input);
}

double square_scale(double input) {
  return copysign(pow(input, 2), input);
}

// Regular tank drive with square scaling
void tank_drive() {
  double left = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double right = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
  left_side.move_voltage(left * 12000);
  right_side.move_voltage(right * 12000);
}
// Regular arcade drive with square scaling
void arcade_drive() {
  double forward = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double turn = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)));
  left_side.move_voltage((forward + turn) * 12000);
  right_side.move_voltage((forward - turn) * 12000);
}
// Hybrid arcade drive with square scaling
void hybrid_drive() {
  double forward = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
  double turn = square_scale(normalize_joystick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
  left_side.move_voltage((forward + turn) * 12000);
  right_side.move_voltage((forward - turn) * 12000);
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

void shoot() {
    while (cata_limit.get_value() == 0) {
        catapult.move_relative(200, 100);
    }
    catapult.move_relative(200, 100);
}

void controls() {
	initTracker();

    left_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_top_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_front_bottom_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	left_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_back_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while(1) {
        tank_drive();

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(12000);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(-12000);
        } else {
            intake.brake();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            shoot();
        } else {
            catapult.brake();
        }

        // update_position();
        // pros::lcd::set_text(3, "X position: " + std::to_string(positionX));
        // pros::lcd::set_text(4, "Y position: " + std::to_string(positionY));
        pros::lcd::set_text(3, "X position: " + std::to_string(transverseEncoder.get_value()));
        pros::lcd::set_text(4, "Y position: " + std::to_string(radialEncoder.get_value()));

        pros::delay(20);
    }
}