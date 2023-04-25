#include "intake.h"
#include "robot.h"

bool mag_piston_state = false;

int discs_in_mag = 0;
int last_disc_dist = 0;
int rai_counter = 0;

void init_intake() {
  set_mag_piston(false);
}

void intake() {
  // make sure mag piston is not engaged
  if (mag_piston_state) {
    toggle_mag_piston();
  }
  intake_mtr.move_voltage(12000);
  // below is logic to sense disc and spin the rai motor when needed
  // also updates discs_in_mag variable
  int distance = disc_dist.get();
  if (distance < 15) {
    rai_counter = 21;
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
  // make sure mag piston is not engaged
  if (mag_piston_state) {
    toggle_mag_piston();
  }
  intake_mtr.move_voltage(-6500);
  // below is logic to sense if disc is being outtaked
  // updates discs_in_mag variable accordingly
  int distance = disc_dist.get();
  if (distance - last_disc_dist > 50) {
    discs_in_mag--;
  }
  last_disc_dist = distance;
}

void toggle_mag_piston() {
  set_mag_piston(!mag_piston_state);
}

void set_mag_piston(bool value) {
  mag_piston_state = value;
  mag_piston.set_value(mag_piston_state);
}

void reset_discs_in_mag() {
  discs_in_mag = 0;
}
int get_discs_in_mag() {
  return discs_in_mag;
}