#include "intake.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <string>

int discs_in_mag = 0;
int last_disc_dist = 0;
int rai_counter = 0;
int jamming_counter = 0;
int JAMMING_LIMIT = 10;
double unjam_start_time = 0.0;


void intake_auton() {
  if (std::abs(intake_mtr.get_actual_velocity()) < 100 && intake_mtr.get_current_draw() > 150) {
    jamming_counter++;
  } else {
    jamming_counter = 0;
  }
  if (jamming_counter > 10) {
    unjam_start_time = pros::millis();
  }
  if (pros::millis() > unjam_start_time + 1000) {
    // normal operation
    intake_mtr.move_voltage(12000);
  } else {
    // jamming, spit it out a bit for a set amount of time
    intake_mtr.move_voltage(-12000);
  }
  // below is logic to sense disc and spin the rai motor when needed
  // also updates discs_in_mag variable
  int distance = disc_dist.get();
  if (distance < 15) {
    rai_counter = (discs_in_mag > 0) ? 28 : 20;
    rai_mtr.move_voltage(8000);
    if (last_disc_dist - distance > 50) {
      discs_in_mag++;
    }
  } else if (rai_counter > 0) {
    rai_counter--;
    rai_mtr.move_voltage(8000);
  } else {
    rai_mtr.move_voltage(0);
  }
  last_disc_dist = distance;
  // pros::lcd::set_text(4, "INTAKE jam counter: " + std::to_string(jamming_counter));
}

void intake() {
  intake_mtr.move_voltage(12000);
  // below is logic to sense disc and spin the rai motor when needed
  // also updates discs_in_mag variable
  int distance = disc_dist.get();
  if (distance < 15) {
    rai_counter = (discs_in_mag > 0) ? 25 : 20;
    rai_mtr.move_voltage(8000);
    if (last_disc_dist - distance > 50) {
      discs_in_mag++;
    }
  } else if (rai_counter > 0) {
    rai_counter--;
    rai_mtr.move_voltage(8000);
  } else {
    rai_mtr.move_voltage(0);
  }
  last_disc_dist = distance;
}


void outtake() {
  intake_mtr.move_voltage(-10000);
  // below is logic to sense if disc is being outtaked
  // updates discs_in_mag variable accordingly
  int distance = disc_dist.get();
  if (distance - last_disc_dist > 50) {
    discs_in_mag--;
  }
  last_disc_dist = distance;
}

void reset_discs_in_mag() {
  discs_in_mag = 0;
}
int get_discs_in_mag() {
  return discs_in_mag;
}