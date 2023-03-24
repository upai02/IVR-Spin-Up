// #include "shooter.h"
#include "../include/shooter.h"
#include "../include/intake.h"
#include <string>
#include "../include/main.h"
#include <pthread.h>
#include <vector>

using std::string;
using std::vector;

int flywheel_voltage = 9000;

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

// adjust robot heading to face goal using camera
// terminates when the heading error is low enough
void auto_aim() {
  // send signal to r-pi to start tracking
  /* NOTE: Trouble sending consistent message to Rasppi for starting algo except when Rasppi is instantiated.
  ** I need to figure out why sending data to Rasppi is possible in 2 way communication thread, but not in any other scenario. 
  ** For now, starting and stopping tracking involves instantiating and destroying a RasppiComms object
  */
  // start_algo();

  // initialize stuff
  pros::c::serctl(SERCTL_DISABLE_COBS,NULL);
  char buffer[256];
	RasppiComms comms = RasppiComms(1);
  // comms.startModel(); // blank function that will get implemented later
  /* size of four since rasppi sends x, y, w, h */
  vector<double> coords(4,0);
	int coords_idx = 0;

  // PID control loop to correct heading error
  get_coords(coords, buffer, coords_idx, comms);
  double error = get_error_from_coords(coords);
  double error_derivative = 0;
  double error_integral = 0;
  double prev_error = 0;
  
  const double error_threshold = 5;
  const double kP = 0.1;
  const double kI = 0.001;
  const double kD = 0;

  while (fabs(error) > error_threshold) {
    get_coords(coords, buffer, coords_idx, comms);
    error = get_error_from_coords(coords);
    error_derivative = error - prev_error;
    error_integral += error;
    prev_error = error;
    double turn = kP * error + kI * error_integral + kD * error_derivative;
    double turn_voltage = turn * 1000;
    left_side.move_voltage(turn_voltage);
    right_side.move_voltage(-turn_voltage);
  }

  // tell r-pi to stop
  // end_algo(); // don't stop the program for now
  /* 'Stopping' the tracking algorithm involves destroying instance of RasppiComms. 
  ** If a reliable manner of sending a message to Rasppi is found, then would be preferred. For now, just create and 
  ** destroy RasppiComms objects
  */
  comms.~RasppiComms();
}
void start_algo() {}
void end_algo() {}
void get_coords(vector<double> &coords, char *buffer, int &coords_idx, RasppiComms &comms) {
  comms.read_256_from_buff(buffer);
	bool done = comms.append_coords(coords, buffer, coords_idx);
}

double get_error_from_coords(vector<double>& coords) {
  // [x, y, width, height]
  // return error in x direction
  double center_frame_x = 360;
  double c_x = coords[0] + coords[2]/2;
  return c_x - center_frame_x;
}