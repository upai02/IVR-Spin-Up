#include "endgame.h"

bool string_release_piston_state = false;
int eg_timer = 0;
/*
drive controller period is 75 seconds.
endgame period is last 10 seconds.
*/

void init_endgame() {
  set_string_release_piston(false);
}

void start_endgame_timer() {
  eg_timer = pros::millis();
}

void release_string() {
  // only activate when there are 20 seconds left
  if (pros::millis() - eg_timer > 55000) {
    toggle_string_release_piston();
  }
}

// piston stuff
void toggle_string_release_piston() {
  set_string_release_piston(!string_release_piston_state);
}

void set_string_release_piston(bool value) {
  string_release_piston_state = value;
  string_release_piston.set_value(string_release_piston_state);
}