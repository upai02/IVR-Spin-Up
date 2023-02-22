#include "controls.h"
#include "pros/misc.h"
#include "robot.h"
#include "drive.h"
#include "intake.h"
#include "shooter.h"

using namespace pros;
using namespace pros::c;

void controls()
{
  master.clear();
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
      stop_flywheel();
    }
    // release discs into flywheel
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      release_discs();
    }
    // if discs in mag is greater than or equal to 2, soft spin
    if (discs_in_mag >= 2) {
      soft_spin();
    }

    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    //   flywheel_rpm += 10;
    // } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    //   flywheel_rpm -= 10;
    // }

    // messing with gps sensor
    // auto gps_data = gps.get_status();

    // pros terminal printing ------------
    std::cout << "target rpm: " << flywheel_rpm << "----- actual: " << flywheel_mtr.get_actual_velocity() << "------ error: " << flywheel_rpm - flywheel_mtr.get_actual_velocity() << std::endl;
    std::cout << "actual flywheel speed rpm: " << flywheel_mtr.get_actual_velocity() << std::endl;
    std::cout << "error rpm: " << flywheel_rpm - flywheel_mtr.get_actual_velocity() << std::endl;
    // std::cout << "GPS: " << "x: " << gps_data.x << " y: " << gps_data.y << std::endl;
    // print to screen ------------
    pros::lcd::print(0, "Drive Mode: %s", get_drive_name().c_str());
    pros::lcd::print(1, "Wheel Value: %d", left_front_mtr.get_position());
    // pros::lcd::print(2, "disc_dist: %lf", disc_dist.get());
    pros::lcd::print(3, "target flywheel rpm: %d", flywheel_rpm);
    pros::lcd::print(4, "actual flywheel rpm: %lf", flywheel_mtr.get_actual_velocity());
    pros::lcd::print(5, "flywheel error rpm: %lf", flywheel_rpm - flywheel_mtr.get_actual_velocity());
    pros::lcd::print(6, "discs in mag: %d", discs_in_mag);
    // pros::lcd::print(7, "GPS -> x: %lf y: %lf", gps_data.x, gps_data.y);
    // print to controller ------------
    master.print(0, 0, "d_m: %s", get_drive_name().c_str());
    master.print(2, 0, "fly_rpm: %d", flywheel_rpm);
    // pros::lcd::print(2, "Vertical Encoder: %lf", (vertical_track.get_value()/5120.0)*360.0);
    // master.print(2, 0, "Vertical Encoder: %lf", vertical_track.get_value());
    pros::delay(20);

  }
}