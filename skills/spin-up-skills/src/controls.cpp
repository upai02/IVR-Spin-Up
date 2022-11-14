#include "controls.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "misc/PositionTracker.cpp"

using namespace pros;

int mode = 2;
double p = 1.0;
PositionTracker positionUpdater = PositionTracker(2.75 * 0.0254 / 2, 2.75 * 0.0254 / 2);

// void tank() {
//     int left = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127) * 100;
//     int right = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) / 127) * 100;
//     left_side.move(left);
//     right_side.move(right);
// }

// void arcade() {
//     int fwd = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127) * 100;
//     int turn = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127) * 100;
//     left_side.move(fwd + turn);
//     right_side.move(fwd - turn);
// }

void mecanum() {
    int fwd = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127) * 100;
    int strafe = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) / 127) * 100;
    int turn = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127) * 100;
    left_front_mtr.move(fwd + strafe + turn);
    right_front_mtr.move(fwd - strafe - turn);
    left_back_mtr.move(fwd - strafe + turn);
    right_back_mtr.move(fwd + strafe - turn);
}

void group_stop(pros::motor_brake_mode_e b) {
    left_front_mtr.set_brake_mode(b);
    left_front_mtr.brake();
    right_front_mtr.set_brake_mode(b);
    right_front_mtr.brake();
    left_back_mtr.set_brake_mode(b);
    left_back_mtr.brake();
    right_back_mtr.set_brake_mode(b);
    right_back_mtr.brake();
}

void move_with_assigned_speed(double xVel, double yVel, double turnVel) {
    left_front_mtr.move_velocity(yVel + xVel - turnVel);
    left_back_mtr.move_velocity(yVel - xVel - turnVel);
    right_front_mtr.move_velocity(yVel - xVel + turnVel);
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

    left_front_mtr.move(power + desiredTranslation);
    left_back_mtr.move(power + desiredTranslation);
    right_front_mtr.move(-power + desiredTranslation);
    right_back_mtr.move(-power + desiredTranslation);
}

void controls() {
	positionUpdater.initTracker();

    while(1) {
        mecanum();

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(12000);
            indexer.move_voltage(12000);
            roller.brake();
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(12000);
            indexer.move_voltage(-12000);
            roller.move_voltage(12000);
        } else {
            intake.brake();
            indexer.brake();
            roller.brake();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            flywheel.move_voltage(7200);
        } else {
            flywheel.brake();
        }

        std::array<double, 2> currentPose = positionUpdater.getPosition(imu.get_heading());
        pros::lcd::set_text(3, "X position: " + std::to_string(currentPose[0]));
        pros::lcd::set_text(4, "Y position: " + std::to_string(currentPose[1]));

        pros::delay(20);
    }
}