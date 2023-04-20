#include "endgame.h"

bool eg_deploy_piston_state = false;
bool string_release_piston_state = false;
int eg_timer = 0;
/*
drive controller period is 75 seconds.
endgame period is last 10 seconds.
*/

void init_endgame() {
  set_eg_deploy_piston(false);
  set_string_release_piston(false);
}

void start_endgame_timer() {
  eg_timer = pros::millis();
}

void deploy_endgame() {
  // only activate when there are 15 seconds left
  if (pros::millis() - eg_timer > 60000) {
    toggle_eg_deploy_piston();
  }
}
void release_string() {
  // only activate when there are 10 seconds left
  if (pros::millis() - eg_timer > 65000) {
    toggle_string_release_piston();
  }
}

// piston stuff
void toggle_eg_deploy_piston() {
  set_eg_deploy_piston(!eg_deploy_piston_state);
}
void toggle_string_release_piston() {
  set_string_release_piston(!string_release_piston_state);
}
void set_eg_deploy_piston(bool value) {
  eg_deploy_piston_state = value;
  eg_deploy_piston.set_value(eg_deploy_piston_state);
}
void set_string_release_piston(bool value) {
  string_release_piston_state = value;
  string_release_piston.set_value(string_release_piston_state);
}