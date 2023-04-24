#include "endgame.h"

bool eg_deploy_piston_state = false;
bool string_release_piston_state = false;
int eg_timer = 0;
/*
drive controller period is 75 seconds.
endgame period is last 10 seconds.
*/

void init_endgame() {
  set_string_release_pistons(false);
}

void start_endgame_timer() {
  eg_timer = pros::millis();
}

void release_string() {
  // only activate when there are 10 seconds left
  if (pros::millis() - eg_timer > 65000) {
    toggle_string_release_pistons();
  }
}

// piston stuff
void toggle_string_release_pistons() {
  set_string_release_pistons(!string_release_piston_state);
}

void set_string_release_pistons(bool value) {
  string_release_piston_state = value;
  string_release_piston_1.set_value(string_release_piston_state);
  string_release_piston_2.set_value(string_release_piston_state);
}