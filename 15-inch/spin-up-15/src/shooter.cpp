#include "shooter.h"
#include "intake.h"
#include "auton.h"
#include "pros/rtos.h"

int flywheel_rpm = close_range_rpm;

void activate_close_range() {
  flywheel_rpm = close_range_rpm;
}
void activate_long_range() {
  flywheel_rpm = long_range_rpm;
}
void set_flywheel_rpm(int rpm) {
  flywheel_rpm = rpm;
}

pros::Task flywheel_task(shoot_thread);

void shoot_thread() {
    while (true) {
      shootPF(flywheel_rpm);
      pros::delay(20);
    }
}


bool soft_spinning = false;
bool flywheel_running = false;

void soft_spin() {
  if (!flywheel_running) {
    flywheel_mtr.move_voltage(5500);
    soft_spinning = true;
  }
}


void run_flywheel() {
  // if running flywheel, then mag should be down to be able to release discs
  soft_spinning = false;
  flywheel_running = true;
  if (!mag_down) {
    toggle_mag_piston();
  }

  // if flywheel is close to target rpm, then rumble controller
  if (abs(flywheel_mtr.get_actual_velocity() - flywheel_rpm) < 20) {
    master.rumble("-");
  }

  flywheel_task.resume();
}

void stop_flywheel() {
  if (!soft_spinning) {
    flywheel_task.suspend();
    flywheel_mtr.move_voltage(0);
    flywheel_running = false;
  }
}

void release_discs() {
  rai_mtr.move_voltage(-12000);
  reset_discs_in_mag();
}

void release_sequence() {

  if (!mag_down) {
    toggle_mag_piston();
  }

  rai_mtr.move_voltage(-12000);
  pros::delay(300);
  discs_in_mag = std::max(0, discs_in_mag - 1);
  rai_mtr.move_voltage(700);
  pros::delay(3000);

  rai_mtr.move_voltage(-12000);
  pros::delay(300);
  discs_in_mag = std::max(0, discs_in_mag - 1);
  rai_mtr.move_voltage(700);
  pros::delay(3000);

  rai_mtr.move_voltage(-12000);
  pros::delay(300);
  discs_in_mag = std::max(0, discs_in_mag - 1);
  rai_mtr.move_voltage(0);
}
