#include "intake.h"

bool intake_out = true;
bool mag_down = true;

int discs_in_mag = 0;

void intake() {
  // add code here to sense disc and run rai motor
  // and update discs_in_mag
  intake_mtr.move_voltage(12000);
  rai_mtr.move_voltage(6000);
}
void outtake() {
  // add code here to sense disc and update discs_in_mag
  intake_mtr.move_voltage(-6500);
}

void toggle_intake_piston() {
  intake_out = !intake_out;
  intake_piston.set_value(intake_out);
}
void toggle_mag_piston() {
  mag_down = !mag_down;
  mag_piston.set_value(mag_down);
}

void reset_discs_in_mag() {
  discs_in_mag = 0;
}
int get_discs_in_mag() {
  return discs_in_mag;
}