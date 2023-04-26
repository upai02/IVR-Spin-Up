#include "shooter.h"
#include "intake.h"
#include "auton.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <vector>
#include <algorithm>

// initialize target flywheel rpm
int target_flywheel_rpm = 0;
// initialize main task to run flywheel
pros::Task flywheel_task(shoot_thread);

// this thread runs the flywheel at the target flywheel rpm
void shoot_thread() {
  while (true) {
    shootPF(target_flywheel_rpm);
    pros::delay(20);
  }
}

void activate_close_range() {
  target_flywheel_rpm = close_range_rpm;
}
void activate_long_range() {
  target_flywheel_rpm = long_range_rpm;
}
void set_flywheel_rpm(int rpm) {
  target_flywheel_rpm = rpm;
}

// returns avg rpm from both flywheel motor encoders
double get_flywheel_rpm() {
  std::vector<double> velocities = flywheel.get_actual_velocities();
  double avg = 0.0;
  for (int i = 0; i < velocities.size(); i++) {
    avg += velocities[i];
  }
  return avg / velocities.size();
}


bool soft_spinning = false; // is flywheel soft spinning?
bool flywheel_running = false; // is flywheel is running for shooting discs?

// gets the flywheel spinning while there are discs in the mag
// this is used to get the flywheel spinning before shooting
// aka less spin up time
void soft_spin() {
  if (!flywheel_running) {
    flywheel.move_voltage(std::min(8000, 4000 * discs_in_mag));
    soft_spinning = true;
  }
}

// runs flywheel when shooting discs (mostly used in teleop)
void run_flywheel() {
  // if running flywheel, then mag should be down to be able to release discs
  soft_spinning = false;
  flywheel_running = true;

  // if flywheel is close to target rpm, then rumble controller
  if (abs(get_flywheel_rpm() - target_flywheel_rpm) < 6) {
    master.rumble("-");
  }

  flywheel_task.resume();
}

// stops flywheel and flywheel task and assigns 0 voltage
void stop_flywheel() {
  if (!soft_spinning) {
    flywheel_task.suspend();
    flywheel.move_voltage(0);
    flywheel_running = false;
  }
}

// release discs from mag into shooter
void release_discs() {
  rai_mtr.move_voltage(-9000);
  reset_discs_in_mag();
}

// auton release sequence for getting discs through mag/shooter
void release_sequence() {
  while (get_flywheel_rpm() < 300) {
    pros::delay(20);
  }

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
