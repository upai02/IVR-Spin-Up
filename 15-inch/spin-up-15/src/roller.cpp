#include "roller.h"


void spin_roller() {
  rai_mtr.move_voltage(12000);
}

void stop_roller() {
  rai_mtr.move_voltage(0);
}

void spin_roller(double rotations_to_spin) {
  rai_mtr.move_relative(rotations_to_spin * 360, 400);
}