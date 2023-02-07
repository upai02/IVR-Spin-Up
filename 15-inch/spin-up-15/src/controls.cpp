#include "controls.h"
#include "pros/misc.h"

using namespace pros;

int mode = 0;

void tank() {
    int left = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127) * 100;
    int right = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) / 127) * 100;
    left_side.move_voltage(left);
    right_side.move_voltage(right);
}

void arcade() {
    int fwd = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127) * 100;
    int turn = ((std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127) * 100;
    left_side.move_voltage(fwd + turn);
    right_side.move_voltage(fwd - turn);
}

void controls() {
    while(1) {
        if (mode == 0) {
            tank();
        } else if (mode == 1) {
            arcade();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            mode = 0;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            mode = 1;
        }

        pros::delay(20);
    }
}