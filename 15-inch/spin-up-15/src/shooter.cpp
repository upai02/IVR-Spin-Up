#include "shooter.h"
#include "intake.h"
#include "auton.h"
#include "pros/rtos.h"
#include <vector>

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

double get_flywheel_rpm() {
  std::vector<double> velocities = flywheel.get_actual_velocities();
  double avg = 0.0;
  for (int i = 0; i < velocities.size(); i++) {
    avg += velocities[i];
  }
  return avg / velocities.size();
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
    flywheel.move_voltage(8000);
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
  if (abs(get_flywheel_rpm() - flywheel_rpm) < 20) {
    master.rumble("-");
  }

  flywheel_task.resume();
}

void stop_flywheel() {
  if (!soft_spinning) {
    flywheel_task.suspend();
    flywheel.move_voltage(0);
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

  // while (flywheel_mtr.get_actual_velocity() < 300) {
  //   pros::delay(20);
  // }

  pros::delay(500);
  rai_mtr.move_voltage(1200);
  pros::delay(100);
  rai_mtr.move_voltage(0);

  pros::delay(1000);
  rai_mtr.move_voltage(-12000);
  pros::delay(300);
  discs_in_mag = std::max(0, discs_in_mag - 1);
  rai_mtr.move_voltage(1200);
  pros::delay(2000);

  rai_mtr.move_voltage(-12000);
  pros::delay(300);
  discs_in_mag = std::max(0, discs_in_mag - 1);
  rai_mtr.move_voltage(1200);
  pros::delay(2000);

  rai_mtr.move_voltage(-12000);
  pros::delay(800);
  discs_in_mag = std::max(0, discs_in_mag - 1);
  rai_mtr.move_voltage(0);
}
