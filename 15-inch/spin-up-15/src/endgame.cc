#include "endgame.h"

bool endgame_released = false;

void activate_endgame() {
  endgame_released = !endgame_released;
  endgame_piston.set_value(endgame_released);
}