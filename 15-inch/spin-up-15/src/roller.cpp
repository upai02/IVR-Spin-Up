#include "roller.h"

void init_roller() {
  roller_opt.set_led_pwm(100);
  roller_opt.disable_gesture();
}

void spin_roller() {
  rai_mtr.move_voltage(12000);
}

void spin_roller_to_hue(double lower_hue, double upper_hue) {
  while (roller_opt.get_hue() < lower_hue || roller_opt.get_hue() > upper_hue) {
    spin_roller();
  }
  rai_mtr.move_voltage(0);
}