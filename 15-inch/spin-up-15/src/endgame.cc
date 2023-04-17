#include "endgame.h"

bool endgame_released = false;

void activate_endgame() {
  endgame_released = !endgame_released;
  blocker_piston.set_value(endgame_released);
}

void init_endgame(bool value) {
  endgame_released = value;
  blocker_piston.set_value(endgame_released);
}