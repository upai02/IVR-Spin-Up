#include "shooter.h"
#include "intake.h"

void activate_close_range() {
  flywheel_voltage = close_range_voltage;
}
void activate_long_range() {
  flywheel_voltage = long_range_voltage;
}

void release_discs() {
  rai_mtr.move_voltage(-12000);
  reset_discs_in_mag();
}

void run_flywheel() {
  flywheel_mtr.move_voltage(flywheel_voltage);
}