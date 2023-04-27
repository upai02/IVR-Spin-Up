#include "controls.h"
#include "pros/misc.h"
#include "robot.h"
#include "drive.h"
#include "intake.h"
#include "roller.h"
#include "endgame.h"
#include "shooter.h"
#include "auton.h"
#include "movement_tank.h"
#include "vision_tracking.h"

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
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      spin_roller();
    } else {
      intake_mtr.move_voltage(0);
      rai_mtr.move_voltage(0);
    }

    // toggle shooter mode
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      activate_close_range();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      activate_long_range();
      // deploy_endgame();
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

    // if discs in mag is greater than 0, soft spin
    // note that there is a bug where if we outtake discs after they've been detected
    // the discs_in_mag variables does not decrement.
    if (discs_in_mag > 0) {
      soft_spin();
    }

    // endgame!
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      release_string();
    }

    // // angle changer
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    //   toggle_angle_changer();
    // }

    // auto-aim
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    //   auto_aim_task.resume();
    // } else {
    //   auto_aim_task.suspend();
    // }

    // hold button - needs more testing/debugging
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    //   turnToPoint();
    // }
  
    // turn pid test
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    //   turnPID(90);
    // }
    // pros::lcd::print(6, "heading: %f", imu.get_heading());


    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    //   target_flywheel_rpm += 10;
    // } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    //   target_flywheel_rpm -= 10;
    // }

    // pros terminal printing ------------
    // std::cout << "target rpm: " << flywheel_rpm << "----- actual: " << flywheel_mtr.get_actual_velocity() << "------ error: " << flywheel_rpm - flywheel_mtr.get_actual_velocity() << std::endl;
    // std::cout << "actual flywheel speed rpm: " << flywheel_mtr.get_actual_velocity() << std::endl;
    // std::cout << "error rpm: " << flywheel_rpm - flywheel_mtr.get_actual_velocity() << std::endl;
    // print to screen ------------
    // pros::lcd::print(0, "Drive Mode: %s", get_drive_name().c_str());
    // pros::lcd::print(1, "Wheel Value: %d", left_front_mtr.get_position());
    // // pros::lcd::print(2, "disc_dist: %lf", disc_dist.get());
    // pros::lcd::print(3, "target flywheel rpm: %d", flywheel_rpm);
    // pros::lcd::print(4, "actual flywheel rpm: %lf", flywheel_mtr.get_actual_velocity());
    // pros::lcd::print(5, "flywheel error rpm: %lf", flywheel_rpm - flywheel_mtr.get_actual_velocity());
    // pros::lcd::print(6, "discs in mag: %d", discs_in_mag);
    // print to controller ------------
    std::string controller_text = "d: " + get_drive_name() + " s: " + get_rpm_state_string();
    master.print(0, 0, controller_text.c_str());
    pros::lcd::print(1, "Flywheel RPM: %lf", get_flywheel_rpm());
    pros::lcd::print(2, "target rpm: %d", target_flywheel_rpm);
    pros::lcd::print(3, "discs in mag: %d", discs_in_mag);
    // master.print(2, 0, "fly_rpm: %d", flywheel_rpm);
    // pros::lcd::print(2, "Vertical Encoder: %lf", (vertical_track.get_value()/5120.0)*360.0);
    // master.print(2, 0, "Vertical Encoder: %lf", vertical_track.get_value());
    pros::delay(20);

  }
}