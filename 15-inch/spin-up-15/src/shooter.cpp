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
    pros::c::delay(20);
  }
}


bool soft_spinning = false;
bool flywheel_running = false;

void soft_spin() {
  if (!flywheel_running) {
    flywheel_mtr.move_voltage(2000);
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

