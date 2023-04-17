#include "intake.h"
#include "robot.h"

bool intake_out = true;
bool mag_down = false;

int discs_in_mag = 0;
int last_disc_dist = 0;

int rai_counter = 0;

void intake() {
  // add code here to sense disc and run rai motor
  // and update discs_in_mag
  // make sure that mag is up before running intake
  if (mag_down) {
    toggle_mag_piston();
  }
  intake_mtr.move_voltage(12000);
  int distance = disc_dist.get();
  if (distance < 15) {
    rai_counter = 15;
    rai_mtr.move_voltage(6000);
    if (last_disc_dist - distance > 50) {
      discs_in_mag++;
    }
  } else if (rai_counter > 0) {
    rai_counter--;
    rai_mtr.move_voltage(6000);
  } else {
    rai_mtr.move_voltage(0);
  }
  last_disc_dist = distance;
}
void outtake() {
  // add code here to sense disc and update discs_in_mag
  if (mag_down) {
    toggle_mag_piston();
  }
  intake_mtr.move_voltage(-6500);
  int distance = disc_dist.get();
  if (distance - last_disc_dist > 50) {
    discs_in_mag--;
  }
  last_disc_dist = distance;
}

// void toggle_intake_piston() {
//   intake_out = !intake_out;
//   intake_piston.set_value(intake_out);
// }
void toggle_mag_piston() {
  mag_down = !mag_down;
  mag_piston.set_value(mag_down);
}
// void set_intake_piston(bool value) {
//   intake_out = !value;
//   intake_piston.set_value(intake_out);
// }
void set_mag_piston(bool value) {
  mag_down = value;
  mag_piston.set_value(mag_down);
}

void reset_discs_in_mag() {
  discs_in_mag = 0;
}
int get_discs_in_mag() {
  return discs_in_mag;
}