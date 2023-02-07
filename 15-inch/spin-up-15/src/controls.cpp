#include "controls.h"
#include "pros/misc.h"
#include "robot.h"
#include "drive.h"
#include "intake.h"
#include "shooter.h"

using namespace pros;

void controls()
{
  while (1)
  {
    op_drive();
    // toggle drive mode
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      toggle_drive_mode();
    }
    // run intake
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake();
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      outtake();
    } else {
      intake_mtr.move_voltage(0);
      rai_mtr.move_voltage(0);
    }
    // intake pistons
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      toggle_intake_piston();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      toggle_mag_piston();
    }
    // toggle shooter mode
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      activate_close_range();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      activate_long_range();
    }
    // run flywheel
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      run_flywheel();
    } else {
      flywheel_mtr.move_voltage(0);
    }
    // release discs into flywheel
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      release_discs();
    }


    // print to screen
    pros::lcd::print(0, "Drive Mode: %s", get_drive_name().c_str());
    master.print(0, 0, "Drive Mode: %s", get_drive_name().c_str());
    pros::lcd::print(1, "Flywheel Voltage: %d", flywheel_voltage);
    master.print(1, 0, "Flywheel Voltage: %d", flywheel_voltage);

    pros::delay(20);
  }
}